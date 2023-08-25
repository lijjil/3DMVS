#include "dialog.h"
#include "ui_dialog.h"


#include "openMVG/cameras/cameras.hpp"
#include "openMVG/exif/exif_IO_EasyExif.hpp"
#include "openMVG/exif/sensor_width_database/ParseDatabase.hpp"
#include "openMVG/geodesy/geodesy.hpp"
#include "openMVG/image/image_io.hpp"
#include "openMVG/numeric/eigen_alias_definition.hpp"
#include "openMVG/sfm/sfm_data.hpp"             //
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/sfm_data_utils.hpp"
#include "openMVG/sfm/sfm_view.hpp"
#include "openMVG/sfm/sfm_view_priors.hpp"
#include "openMVG/types.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/progress/progress_display.hpp"

#include <cereal/archives/json.hpp>

#include "openMVG/features/akaze/image_describer_akaze_io.hpp"

#include "openMVG/features/sift/SIFT_Anatomy_Image_Describer_io.hpp"
#include "openMVG/image/image_io.hpp"
#include "openMVG/features/regions_factory_io.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/system/timer.hpp"


#include "nonFree/sift/SIFT_describer_io.hpp"

#include <cereal/details/helpers.hpp>

#include "openMVG/graph/graph.hpp"
#include "openMVG/graph/graph_stats.hpp"
#include "openMVG/features/akaze/image_describer_akaze.hpp"
#include "openMVG/features/descriptor.hpp"
#include "openMVG/features/feature.hpp"
#include "openMVG/matching/indMatch.hpp"
#include "openMVG/matching/indMatch_utils.hpp"
#include "openMVG/matching_image_collection/Matcher_Regions.hpp"
#include "openMVG/matching_image_collection/Cascade_Hashing_Matcher_Regions.hpp"
#include "openMVG/matching_image_collection/GeometricFilter.hpp"
#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_regions_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_regions_provider_cache.hpp"
#include "openMVG/matching_image_collection/F_ACRobust.hpp"
#include "openMVG/matching_image_collection/E_ACRobust.hpp"
#include "openMVG/matching_image_collection/E_ACRobust_Angular.hpp"
#include "openMVG/matching_image_collection/Eo_Robust.hpp"
#include "openMVG/matching_image_collection/H_ACRobust.hpp"
#include "openMVG/matching_image_collection/Pair_Builder.hpp"
#include "openMVG/matching/pairwiseAdjacencyDisplay.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/stl/stl.hpp"
#include "openMVG/system/timer.hpp"

#include "openMVG/cameras/Camera_Common.hpp"
#include "openMVG/cameras/Cameras_Common_command_line_helper.hpp"
#include "openMVG/sfm/pipelines/sequential/sequential_SfM.hpp"
#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_matches_provider.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/sfm_report.hpp"
#include "openMVG/sfm/sfm_view.hpp"
#include "openMVG/system/timer.hpp"
#include "openMVG/types.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <atomic>
#include <cstdlib>
#include <iostream>


#ifdef OPENMVG_USE_OPENMP
#include <omp.h>
#endif

using namespace openMVG;
using namespace openMVG::image;
using namespace openMVG::features;
using namespace openMVG::sfm;
using namespace openMVG::cameras;
using namespace openMVG::exif;
using namespace openMVG::geodesy;
using namespace std;
using namespace openMVG::matching;
using namespace openMVG::robust;
using namespace openMVG::matching_image_collection;

bool checkIntrinsicStringValidity(const std::string & Kmatrix, double & focal, double & ppx, double & ppy);
std::pair<bool, Vec3> checkGPS(const std::string & filename, const int & GPS_to_XYZ_method);
std::pair<bool, Vec3> checkPriorWeightsString(const std::string &sWeights);
features::EDESCRIBER_PRESET stringToEnum(const std::string & sPreset);
bool computeIndexFromImageNames(
  const SfM_Data & sfm_data,
  const std::pair<std::string,std::string>& initialPairName,
  Pair& initialPairIndex);



using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

enum EGeometricModel
{
  FUNDAMENTAL_MATRIX = 0,
  ESSENTIAL_MATRIX   = 1,
  HOMOGRAPHY_MATRIX  = 2,
  ESSENTIAL_MATRIX_ANGULAR = 3,
  ESSENTIAL_MATRIX_ORTHO = 4,
  ESSENTIAL_MATRIX_UPRIGHT = 5
};

enum EPairMode
{
  PAIR_EXHAUSTIVE = 0,
  PAIR_CONTIGUOUS = 1,
  PAIR_FROM_FILE  = 2
};

Dialog::Dialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::Dialog)
{
    ui->setupUi(this);
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->qvtkWidget->GetInteractor(),ui->qvtkWidget->GetRenderWindow());

}

Dialog::~Dialog()
{
    delete ui;
}


bool checkIntrinsicStringValidity(const std::string & Kmatrix, double & focal, double & ppx, double & ppy)
{
  std::vector<std::string> vec_str;
  stl::split(Kmatrix, ';', vec_str);    //
  if (vec_str.size() != 9)  {
    std::cerr << "\n Missing ';' character" << std::endl;
    return false;
  }

  // Check that all K matrix value are valid numbers
  for (size_t i = 0; i < vec_str.size(); ++i) {
    double readvalue = 0.0;
    std::stringstream ss;
    ss.str(vec_str[i]);
    if (! (ss >> readvalue) )  {
      std::cerr << "\n Used an invalid not a number character" << std::endl;
      return false;
    }
    if (i==0) focal = readvalue;
    if (i==2) ppx = readvalue;
    if (i==5) ppy = readvalue;
  }
  return true;
}


std::pair<bool, Vec3> checkGPS(const std::string & filename,const int & GPS_to_XYZ_method = 0)
{
  std::pair<bool, Vec3> val(false, Vec3::Zero());
  std::unique_ptr<Exif_IO> exifReader(new Exif_IO_EasyExif);
  if (exifReader)
  {
    // Try to parse EXIF metada & check existence of EXIF data
    if ( exifReader->open( filename ) && exifReader->doesHaveExifInfo() )
    {
      // Check existence of GPS coordinates
      double latitude, longitude, altitude;
      if ( exifReader->GPSLatitude( &latitude ) &&
           exifReader->GPSLongitude( &longitude ) &&
           exifReader->GPSAltitude( &altitude ) )
      {
        // Add ECEF or UTM XYZ position to the GPS position array
        val.first = true;
        switch (GPS_to_XYZ_method)      // GPS 的数据类型转换
        {
          case 1:
            val.second = lla_to_utm( latitude, longitude, altitude );
            break;
          case 0:
          default:
            val.second = lla_to_ecef( latitude, longitude, altitude );
            break;
        }
      }
    }
  }
  return val;
}

/// Check string of prior weights
std::pair<bool, Vec3> checkPriorWeightsString(const std::string &sWeights)
{
  std::pair<bool, Vec3> val(true, Vec3::Zero());
  std::vector<std::string> vec_str;
  stl::split(sWeights, ';', vec_str);
  if (vec_str.size() != 3)
  {
    std::cerr << "\n Missing ';' character in prior weights" << std::endl;
    val.first = false;
  }
  // Check that all weight values are valid numbers
  for (size_t i = 0; i < vec_str.size(); ++i)
  {
    double readvalue = 0.0;
    std::stringstream ss;
    ss.str(vec_str[i]);
    if (! (ss >> readvalue) )  {
      std::cerr << "\n Used an invalid not a number character in local frame origin" << std::endl;
      val.first = false;
    }
    val.second[i] = readvalue;
  }
  return val;
}

void Dialog::on_pushButton_clicked()
{
    viewer->setBackgroundColor (120, 120, 120);
    PointCloudT::Ptr cloud(new PointCloudT);

    // 读取点云
    pcl::io::loadPLYFile("scene_dense.ply", *cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "sample cloud");
    viewer->resetCamera();
    ui->qvtkWidget->update();

}

void Dialog::on_pushButton_4_clicked()
{
    CmdLine cmd;

    std::string sImageDir,
      sfileDatabase = "",
      sOutputDir = "",
      sKmatrix;

    std::string sPriorWeights;
    std::pair<bool, Vec3> prior_w_info(false, Vec3(1.0,1.0,1.0));

    int i_User_camera_model = PINHOLE_CAMERA_RADIAL3;

    bool b_Group_camera_model = true;

    int i_GPS_XYZ_method = 0;

    double focal_pixels = -1.0;

    cmd.add( make_option('i', sImageDir, "imageDirectory") );
    cmd.add( make_option('d', sfileDatabase, "sensorWidthDatabase") );
    cmd.add( make_option('o', sOutputDir, "outputDirectory") );
    cmd.add( make_option('f', focal_pixels, "focal") );
    cmd.add( make_option('k', sKmatrix, "intrinsics") );
    cmd.add( make_option('c', i_User_camera_model, "camera_model") );
    cmd.add( make_option('g', b_Group_camera_model, "group_camera_model") );
    cmd.add( make_switch('P', "use_pose_prior") );
    cmd.add( make_option('W', sPriorWeights, "prior_weights"));
    cmd.add( make_option('m', i_GPS_XYZ_method, "gps_to_xyz_method") );


    // Expected properties for each image
    double width = -1, height = -1, focal = -1, ppx = -1,  ppy = -1;

    const EINTRINSIC e_User_camera_model = EINTRINSIC(i_User_camera_model);

    if ( !stlplus::folder_exists( sImageDir ) )
    {
      std::cerr << "\nThe input directory doesn't exist" << std::endl;
    }

    if (sOutputDir.empty())
    {
      std::cerr << "\nInvalid output directory" << std::endl;
    }

    if ( !stlplus::folder_exists( sOutputDir ) )
    {
      if ( !stlplus::folder_create( sOutputDir ))
      {
        std::cerr << "\nCannot create output directory" << std::endl;
      }
    }

    if (sKmatrix.size() > 0 &&
      !checkIntrinsicStringValidity(sKmatrix, focal, ppx, ppy) )
    {
      std::cerr << "\nInvalid K matrix input" << std::endl;
    }

    if (sKmatrix.size() > 0 && focal_pixels != -1.0)
    {
      std::cerr << "\nCannot combine -f and -k options" << std::endl;
    }

    std::vector<Datasheet> vec_database;
    if (!sfileDatabase.empty())
    {
      if ( !parseDatabase( sfileDatabase, vec_database ) )
      {
        std::cerr
         << "\nInvalid input database: " << sfileDatabase
         << ", please specify a valid file." << std::endl;
      }
    }

    // Check if prior weights are given
    if (cmd.used('P') && !sPriorWeights.empty())
    {
      prior_w_info = checkPriorWeightsString(sPriorWeights);
    }
    else if (cmd.used('P'))
    {
      prior_w_info.first = true;
    }

    std::vector<std::string> vec_image = stlplus::folder_files( sImageDir );
    std::sort(vec_image.begin(), vec_image.end());

    // Configure an empty scene with Views and their corresponding cameras
    SfM_Data sfm_data;
    sfm_data.s_root_path = sImageDir; // Setup main image root_path
    Views & views = sfm_data.views;
    Intrinsics & intrinsics = sfm_data.intrinsics;

    C_Progress_display my_progress_bar( vec_image.size(),
        std::cout, "\n- Image listing -\n" );
    std::ostringstream error_report_stream;
    for ( std::vector<std::string>::const_iterator iter_image = vec_image.begin();
      iter_image != vec_image.end();
      ++iter_image, ++my_progress_bar )
    {
      // Read meta data to fill camera parameter (w,h,focal,ppx,ppy) fields.
      width = height = ppx = ppy = focal = -1.0;

      const std::string sImageFilename = stlplus::create_filespec( sImageDir, *iter_image );
      const std::string sImFilenamePart = stlplus::filename_part(sImageFilename);

      // Test if the image format is supported:
      if (openMVG::image::GetFormat(sImageFilename.c_str()) == openMVG::image::Unknown)
      {
        error_report_stream
            << sImFilenamePart << ": Unkown image file format." << "\n";
        continue; // image cannot be opened
      }

      if (sImFilenamePart.find("mask.png") != std::string::npos
         || sImFilenamePart.find("_mask.png") != std::string::npos)
      {
        error_report_stream
            << sImFilenamePart << " is a mask image" << "\n";
        continue;
      }

      ImageHeader imgHeader;
      if (!openMVG::image::ReadImageHeader(sImageFilename.c_str(), &imgHeader))
        continue; // image cannot be read

      width = imgHeader.width;
      height = imgHeader.height;
      ppx = width / 2.0;
      ppy = height / 2.0;


      // Consider the case where the focal is provided manually
      if (sKmatrix.size() > 0) // Known user calibration K matrix
      {
        if (!checkIntrinsicStringValidity(sKmatrix, focal, ppx, ppy))
          focal = -1.0;
      }
      else // User provided focal length value
        if (focal_pixels != -1 )
          focal = focal_pixels;

      // If not manually provided or wrongly provided
      if (focal == -1)
      {
        std::unique_ptr<Exif_IO> exifReader(new Exif_IO_EasyExif);
        exifReader->open( sImageFilename );

        const bool bHaveValidExifMetadata =
          exifReader->doesHaveExifInfo()
          && !exifReader->getModel().empty();

        if (bHaveValidExifMetadata) // If image contains meta data
        {
          const std::string sCamModel = exifReader->getModel();

          // Handle case where focal length is equal to 0
          if (exifReader->getFocal() == 0.0f)
          {
            error_report_stream
              << stlplus::basename_part(sImageFilename) << ": Focal length is missing." << "\n";
            focal = -1.0;
          }
          else
          // Create the image entry in the list file
          {
            Datasheet datasheet;
            if ( getInfo( sCamModel, vec_database, datasheet ))
            {
              // The camera model was found in the database so we can compute it's approximated focal length
              const double ccdw = datasheet.sensorSize_;
              focal = std::max ( width, height ) * exifReader->getFocal() / ccdw;
            }
            else
            {
              error_report_stream
                << stlplus::basename_part(sImageFilename)
                << "\" model \"" << sCamModel << "\" doesn't exist in the database" << "\n"
                << "Please consider add your camera model and sensor width in the database." << "\n";
            }
          }
        }
      }
      // Build intrinsic parameter related to the view
      std::shared_ptr<IntrinsicBase> intrinsic;

      if (focal > 0 && ppx > 0 && ppy > 0 && width > 0 && height > 0)
      {
        // Create the desired camera type
        switch (e_User_camera_model)
        {
          case PINHOLE_CAMERA:
            intrinsic = std::make_shared<Pinhole_Intrinsic>
              (width, height, focal, ppx, ppy);
          break;
          case PINHOLE_CAMERA_RADIAL1:
            intrinsic = std::make_shared<Pinhole_Intrinsic_Radial_K1>
              (width, height, focal, ppx, ppy, 0.0); // setup no distortion as initial guess
          break;
          case PINHOLE_CAMERA_RADIAL3:
            intrinsic = std::make_shared<Pinhole_Intrinsic_Radial_K3>
              (width, height, focal, ppx, ppy, 0.0, 0.0, 0.0);  // setup no distortion as initial guess
          break;
          case PINHOLE_CAMERA_BROWN:
            intrinsic = std::make_shared<Pinhole_Intrinsic_Brown_T2>
              (width, height, focal, ppx, ppy, 0.0, 0.0, 0.0, 0.0, 0.0); // setup no distortion as initial guess
          break;
          case PINHOLE_CAMERA_FISHEYE:
            intrinsic = std::make_shared<Pinhole_Intrinsic_Fisheye>
              (width, height, focal, ppx, ppy, 0.0, 0.0, 0.0, 0.0); // setup no distortion as initial guess
          break;
          case CAMERA_SPHERICAL:
             intrinsic = std::make_shared<Intrinsic_Spherical>
               (width, height);
          break;
          default:
            std::cerr << "Error: unknown camera model: " << (int) e_User_camera_model << std::endl;
        }
      }

      // Build the view corresponding to the image
      const std::pair<bool, Vec3> gps_info = checkGPS(sImageFilename, i_GPS_XYZ_method);
      if (gps_info.first && cmd.used('P'))
      {
        ViewPriors v(*iter_image, views.size(), views.size(), views.size(), width, height);

        // Add intrinsic related to the image (if any)
        if (intrinsic == nullptr)
        {
          //Since the view have invalid intrinsic data
          // (export the view, with an invalid intrinsic field value)
          v.id_intrinsic = UndefinedIndexT;
        }
        else
        {
          // Add the defined intrinsic to the sfm_container
          intrinsics[v.id_intrinsic] = intrinsic;
        }

        v.b_use_pose_center_ = true;
        v.pose_center_ = gps_info.second;
        // prior weights
        if (prior_w_info.first == true)
        {
          v.center_weight_ = prior_w_info.second;
        }

        // Add the view to the sfm_container
        views[v.id_view] = std::make_shared<ViewPriors>(v);
      }
      else
      {
        View v(*iter_image, views.size(), views.size(), views.size(), width, height);

        // Add intrinsic related to the image (if any)
        if (intrinsic == nullptr)
        {
          //Since the view have invalid intrinsic data
          // (export the view, with an invalid intrinsic field value)
          v.id_intrinsic = UndefinedIndexT;
        }
        else
        {
          // Add the defined intrinsic to the sfm_container
          intrinsics[v.id_intrinsic] = intrinsic;
        }

        // Add the view to the sfm_container
        views[v.id_view] = std::make_shared<View>(v);
      }
    }

    // Display saved warning & error messages if any.
    if (!error_report_stream.str().empty())
    {
      std::cerr
        << "\nWarning & Error messages:" << std::endl
        << error_report_stream.str() << std::endl;
    }

    // Group camera that share common properties if desired (leads to more faster & stable BA).
    if (b_Group_camera_model)
    {
      GroupSharedIntrinsics(sfm_data);
    }

    // Store SfM_Data views & intrinsic data
    if (!Save(
      sfm_data,
      stlplus::create_filespec( sOutputDir, "sfm_data.json" ).c_str(),
      ESfM_Data(VIEWS|INTRINSICS)))
    {
    }

    std::cout << std::endl
      << "SfMInit_ImageListing report:\n"
      << "listed #File(s): " << vec_image.size() << "\n"
      << "usable #File(s) listed in sfm_data: " << sfm_data.GetViews().size() << "\n"
      << "usable #Intrinsic(s) listed in sfm_data: " << sfm_data.GetIntrinsics().size() << std::endl;

}

void Dialog::on_pushButton_3_clicked()
{
    CmdLine cmd;

    std::string sSfM_Data_Filename;
    std::string sOutDir = "";
    bool bUpRight = false;
    std::string sImage_Describer_Method = "SIFT";
    bool bForce = false;
    std::string sFeaturePreset = "";
  #ifdef OPENMVG_USE_OPENMP
    int iNumThreads = 0;
  #endif

    // required
    cmd.add( make_option('i', sSfM_Data_Filename, "input_file") );
    cmd.add( make_option('o', sOutDir, "outdir") );
    // Optional
    cmd.add( make_option('m', sImage_Describer_Method, "describerMethod") );
    cmd.add( make_option('u', bUpRight, "upright") );
    cmd.add( make_option('f', bForce, "force") );
    cmd.add( make_option('p', sFeaturePreset, "describerPreset") );

  #ifdef OPENMVG_USE_OPENMP
    cmd.add( make_option('n', iNumThreads, "numThreads") );
  #endif






    //---------------------------------------
    // a. Load input scene
    //---------------------------------------
    SfM_Data sfm_data;
    if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS|INTRINSICS))) {
      std::cerr << std::endl
        << "The input file \""<< sSfM_Data_Filename << "\" cannot be read" << std::endl;
    }

    // b. Init the image_describer
    // - retrieve the used one in case of pre-computed features
    // - else create the desired one

    using namespace openMVG::features;
    std::unique_ptr<Image_describer> image_describer;

    const std::string sImage_describer = stlplus::create_filespec(sOutDir, "image_describer", "json");
    if (!bForce && stlplus::is_file(sImage_describer))
    {
      // Dynamically load the image_describer from the file (will restore old used settings)
      std::ifstream stream(sImage_describer.c_str());
      if (!stream.is_open())

      try
      {
        cereal::JSONInputArchive archive(stream);
        archive(cereal::make_nvp("image_describer", image_describer));
      }
      catch (const cereal::Exception & e)
      {
        std::cerr << e.what() << std::endl
          << "Cannot dynamically allocate the Image_describer interface." << std::endl;
      }
    }
    else
    {
      // Create the desired Image_describer method.
      // Don't use a factory, perform direct allocation
      if (sImage_Describer_Method == "SIFT")
      {
        image_describer.reset(new SIFT_Image_describer
          (SIFT_Image_describer::Params(), !bUpRight));
      }
      else
      if (sImage_Describer_Method == "SIFT_ANATOMY")
      {
        image_describer.reset(
          new SIFT_Anatomy_Image_describer(SIFT_Anatomy_Image_describer::Params()));
      }
      else
      if (sImage_Describer_Method == "AKAZE_FLOAT")
      {
        image_describer = AKAZE_Image_describer::create
          (AKAZE_Image_describer::Params(AKAZE::Params(), AKAZE_MSURF), !bUpRight);
      }
      else
      if (sImage_Describer_Method == "AKAZE_MLDB")
      {
        image_describer = AKAZE_Image_describer::create
          (AKAZE_Image_describer::Params(AKAZE::Params(), AKAZE_MLDB), !bUpRight);
      }
      if (!image_describer)
      {
        std::cerr << "Cannot create the designed Image_describer:"
          << sImage_Describer_Method << "." << std::endl;
      }
      else
      {
        if (!sFeaturePreset.empty())
        if (!image_describer->Set_configuration_preset(stringToEnum(sFeaturePreset)))
        {
          std::cerr << "Preset configuration failed." << std::endl;
        }
      }

      // Export the used Image_describer and region type for:
      // - dynamic future regions computation and/or loading

      {
        std::ofstream stream(sImage_describer.c_str());

        cereal::JSONOutputArchive archive(stream);
        archive(cereal::make_nvp("image_describer", image_describer));
        auto regionsType = image_describer->Allocate();
        archive(cereal::make_nvp("regions_type", regionsType));
      }
    }

    // Feature extraction routines
    // For each View of the SfM_Data container:
    // - if regions file exists continue,
    // - if no file, compute features
    {
      system::Timer timer;
      Image<unsigned char> imageGray;

      C_Progress_display my_progress_bar(sfm_data.GetViews().size(),
        std::cout, "\n- EXTRACT FEATURES -\n" );

      // Use a boolean to track if we must stop feature extraction
      std::atomic<bool> preemptive_exit(false);
  #ifdef OPENMVG_USE_OPENMP
      const unsigned int nb_max_thread = omp_get_max_threads();

      if (iNumThreads > 0) {
          omp_set_num_threads(iNumThreads);
      } else {
          omp_set_num_threads(nb_max_thread);
      }

      #pragma omp parallel for schedule(dynamic) if (iNumThreads > 0) private(imageGray)
  #endif
      for (int i = 0; i < static_cast<int>(sfm_data.views.size()); ++i)
      {
        Views::const_iterator iterViews = sfm_data.views.begin();
        std::advance(iterViews, i);
        const View * view = iterViews->second.get();
        const std::string
          sView_filename = stlplus::create_filespec(sfm_data.s_root_path, view->s_Img_path),
          sFeat = stlplus::create_filespec(sOutDir, stlplus::basename_part(sView_filename), "feat"),
          sDesc = stlplus::create_filespec(sOutDir, stlplus::basename_part(sView_filename), "desc");

        // If features or descriptors file are missing, compute them
        if (!preemptive_exit && (bForce || !stlplus::file_exists(sFeat) || !stlplus::file_exists(sDesc)))
        {
          if (!ReadImage(sView_filename.c_str(), &imageGray))
            continue;

          //
          // Look if there is occlusion feature mask
          //
          Image<unsigned char> * mask = nullptr; // The mask is null by default

          const std::string
            mask_filename_local =
              stlplus::create_filespec(sfm_data.s_root_path,
                stlplus::basename_part(sView_filename) + "_mask", "png"),
            mask__filename_global =
              stlplus::create_filespec(sfm_data.s_root_path, "mask", "png");

          Image<unsigned char> imageMask;
          // Try to read the local mask
          if (stlplus::file_exists(mask_filename_local))
          {
            if (!ReadImage(mask_filename_local.c_str(), &imageMask))
            {
              std::cerr << "Invalid mask: " << mask_filename_local << std::endl
                        << "Stopping feature extraction." << std::endl;
              preemptive_exit = true;
              continue;
            }
            // Use the local mask only if it fits the current image size
            if (imageMask.Width() == imageGray.Width() && imageMask.Height() == imageGray.Height())
              mask = &imageMask;
          }
          else
          {
            // Try to read the global mask
            if (stlplus::file_exists(mask__filename_global))
            {
              if (!ReadImage(mask__filename_global.c_str(), &imageMask))
              {
                std::cerr << "Invalid mask: " << mask__filename_global << std::endl
                          << "Stopping feature extraction." << std::endl;
                preemptive_exit = true;
                continue;
              }
              // Use the global mask only if it fits the current image size
              if (imageMask.Width() == imageGray.Width() && imageMask.Height() == imageGray.Height())
                mask = &imageMask;
            }
          }

          // Compute features and descriptors and export them to files
          auto regions = image_describer->Describe(imageGray, mask);
          if (regions && !image_describer->Save(regions.get(), sFeat, sDesc)) {
            std::cerr << "Cannot save regions for images: " << sView_filename << std::endl
                      << "Stopping feature extraction." << std::endl;
            preemptive_exit = true;
            continue;
          }
        }
        ++my_progress_bar;
      }
      std::cout << "Task done in (s): " << timer.elapsed() << std::endl;
    }
}


features::EDESCRIBER_PRESET stringToEnum(const std::string & sPreset)
{
  features::EDESCRIBER_PRESET preset;
  if (sPreset == "NORMAL")
    preset = features::NORMAL_PRESET;
  else
  if (sPreset == "HIGH")
    preset = features::HIGH_PRESET;
  else
  if (sPreset == "ULTRA")
    preset = features::ULTRA_PRESET;
  else
    preset = features::EDESCRIBER_PRESET(-1);
  return preset;
}

void Dialog::on_pushButton_2_clicked()
{
    CmdLine cmd;

    std::string sSfM_Data_Filename;
    std::string sMatchesDirectory = "";
    std::string sGeometricModel = "f";
    float fDistRatio = 0.8f;
    int iMatchingVideoMode = -1;
    std::string sPredefinedPairList = "";
    std::string sNearestMatchingMethod = "AUTO";
    bool bForce = false;
    bool bGuided_matching = false;
    int imax_iteration = 2048;
    unsigned int ui_max_cache_size = 0;

    //required
    cmd.add( make_option('i', sSfM_Data_Filename, "input_file") );
    cmd.add( make_option('o', sMatchesDirectory, "out_dir") );
    // Options
    cmd.add( make_option('r', fDistRatio, "ratio") );
    cmd.add( make_option('g', sGeometricModel, "geometric_model") );
    cmd.add( make_option('v', iMatchingVideoMode, "video_mode_matching") );
    cmd.add( make_option('l', sPredefinedPairList, "pair_list") );
    cmd.add( make_option('n', sNearestMatchingMethod, "nearest_matching_method") );
    cmd.add( make_option('f', bForce, "force") );
    cmd.add( make_option('m', bGuided_matching, "guided_matching") );
    cmd.add( make_option('I', imax_iteration, "max_iteration") );
    cmd.add( make_option('c', ui_max_cache_size, "cache_size") );




    EPairMode ePairmode = (iMatchingVideoMode == -1 ) ? PAIR_EXHAUSTIVE : PAIR_CONTIGUOUS;

    if (sPredefinedPairList.length()) {
      ePairmode = PAIR_FROM_FILE;
      if (iMatchingVideoMode>0) {
        std::cerr << "\nIncompatible options: --videoModeMatching and --pairList" << std::endl;
      }
    }

    if (sMatchesDirectory.empty() || !stlplus::is_folder(sMatchesDirectory))  {
      std::cerr << "\nIt is an invalid output directory" << std::endl;
    }

    EGeometricModel eGeometricModelToCompute = FUNDAMENTAL_MATRIX;
    std::string sGeometricMatchesFilename = "";
    switch (sGeometricModel[0])
    {
      case 'f': case 'F':
        eGeometricModelToCompute = FUNDAMENTAL_MATRIX;
        sGeometricMatchesFilename = "matches.f.bin";
      break;
      case 'e': case 'E':
        eGeometricModelToCompute = ESSENTIAL_MATRIX;
        sGeometricMatchesFilename = "matches.e.bin";
      break;
      case 'h': case 'H':
        eGeometricModelToCompute = HOMOGRAPHY_MATRIX;
        sGeometricMatchesFilename = "matches.h.bin";
      break;
      case 'a': case 'A':
        eGeometricModelToCompute = ESSENTIAL_MATRIX_ANGULAR;
        sGeometricMatchesFilename = "matches.f.bin";
      break;
      case 'o': case 'O':
        eGeometricModelToCompute = ESSENTIAL_MATRIX_ORTHO;
        sGeometricMatchesFilename = "matches.o.bin";
      break;
      case 'u': case 'U':
        eGeometricModelToCompute = ESSENTIAL_MATRIX_UPRIGHT;
        sGeometricMatchesFilename = "matches.f.bin";
      break;
      default:
        std::cerr << "Unknown geometric model" << std::endl;
    }

    // -----------------------------
    // - Load SfM_Data Views & intrinsics data
    // a. Compute putative descriptor matches
    // b. Geometric filtering of putative matches
    // + Export some statistics
    // -----------------------------

    //---------------------------------------
    // Read SfM Scene (image view & intrinsics data)
    //---------------------------------------
    SfM_Data sfm_data;
    if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS|INTRINSICS))) {
      std::cerr << std::endl
        << "The input SfM_Data file \""<< sSfM_Data_Filename << "\" cannot be read." << std::endl;
    }

    //---------------------------------------
    // Load SfM Scene regions
    //---------------------------------------
    // Init the regions_type from the image describer file (used for image regions extraction)
    using namespace openMVG::features;
    const std::string sImage_describer = stlplus::create_filespec(sMatchesDirectory, "image_describer", "json");
    std::unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);
    if (!regions_type)
    {
      std::cerr << "Invalid: "
        << sImage_describer << " regions type file." << std::endl;
    }

    //---------------------------------------
    // a. Compute putative descriptor matches
    //    - Descriptor matching (according user method choice)
    //    - Keep correspondences only if NearestNeighbor ratio is ok
    //---------------------------------------

    // Load the corresponding view regions
    std::shared_ptr<Regions_Provider> regions_provider;
    if (ui_max_cache_size == 0)
    {
      // Default regions provider (load & store all regions in memory)
      regions_provider = std::make_shared<Regions_Provider>();
    }
    else
    {
      // Cached regions provider (load & store regions on demand)
      regions_provider = std::make_shared<Regions_Provider_Cache>(ui_max_cache_size);
    }

    // Show the progress on the command line:
    C_Progress_display progress;

    if (!regions_provider->load(sfm_data, sMatchesDirectory, regions_type, &progress)) {
      std::cerr << std::endl << "Invalid regions." << std::endl;
    }

    PairWiseMatches map_PutativesMatches;

    // Build some alias from SfM_Data Views data:
    // - List views as a vector of filenames & image sizes
    std::vector<std::string> vec_fileNames;
    std::vector<std::pair<size_t, size_t>> vec_imagesSize;
    {
      vec_fileNames.reserve(sfm_data.GetViews().size());
      vec_imagesSize.reserve(sfm_data.GetViews().size());
      for (Views::const_iterator iter = sfm_data.GetViews().begin();
        iter != sfm_data.GetViews().end();
        ++iter)
      {
        const View * v = iter->second.get();
        vec_fileNames.push_back(stlplus::create_filespec(sfm_data.s_root_path,
            v->s_Img_path));
        vec_imagesSize.push_back( std::make_pair( v->ui_width, v->ui_height) );
      }
    }

    std::cout << std::endl << " - PUTATIVE MATCHES - " << std::endl;
    // If the matches already exists, reload them
    if (!bForce
          && (stlplus::file_exists(sMatchesDirectory + "/matches.putative.txt")
          || stlplus::file_exists(sMatchesDirectory + "/matches.putative.bin"))
    )
    {
      if (!(Load(map_PutativesMatches, sMatchesDirectory + "/matches.putative.bin") ||
            Load(map_PutativesMatches, sMatchesDirectory + "/matches.putative.txt")) )
      {
        std::cerr << "Cannot load input matches file";
      }
      std::cout << "\t PREVIOUS RESULTS LOADED;"
        << " #pair: " << map_PutativesMatches.size() << std::endl;
    }
    else // Compute the putative matches
    {
      std::cout << "Use: ";
      switch (ePairmode)
      {
        case PAIR_EXHAUSTIVE: std::cout << "exhaustive pairwise matching" << std::endl; break;
        case PAIR_CONTIGUOUS: std::cout << "sequence pairwise matching" << std::endl; break;
        case PAIR_FROM_FILE:  std::cout << "user defined pairwise matching" << std::endl; break;
      }

      // Allocate the right Matcher according the Matching requested method
      std::unique_ptr<Matcher> collectionMatcher;
      if (sNearestMatchingMethod == "AUTO")
      {
        if (regions_type->IsScalar())
        {
          std::cout << "Using FAST_CASCADE_HASHING_L2 matcher" << std::endl;
          collectionMatcher.reset(new Cascade_Hashing_Matcher_Regions(fDistRatio));
        }
        else
        if (regions_type->IsBinary())
        {
          std::cout << "Using BRUTE_FORCE_HAMMING matcher" << std::endl;
          collectionMatcher.reset(new Matcher_Regions(fDistRatio, BRUTE_FORCE_HAMMING));
        }
      }
      else
      if (sNearestMatchingMethod == "BRUTEFORCEL2")
      {
        std::cout << "Using BRUTE_FORCE_L2 matcher" << std::endl;
        collectionMatcher.reset(new Matcher_Regions(fDistRatio, BRUTE_FORCE_L2));
      }
      else
      if (sNearestMatchingMethod == "BRUTEFORCEHAMMING")
      {
        std::cout << "Using BRUTE_FORCE_HAMMING matcher" << std::endl;
        collectionMatcher.reset(new Matcher_Regions(fDistRatio, BRUTE_FORCE_HAMMING));
      }
      else
      if (sNearestMatchingMethod == "HNSWL2")
      {
        std::cout << "Using HNSWL2 matcher" << std::endl;
        collectionMatcher.reset(new Matcher_Regions(fDistRatio, HNSW_L2));
      }
      else
      if (sNearestMatchingMethod == "ANNL2")
      {
        std::cout << "Using ANN_L2 matcher" << std::endl;
        collectionMatcher.reset(new Matcher_Regions(fDistRatio, ANN_L2));
      }
      else
      if (sNearestMatchingMethod == "CASCADEHASHINGL2")
      {
        std::cout << "Using CASCADE_HASHING_L2 matcher" << std::endl;
        collectionMatcher.reset(new Matcher_Regions(fDistRatio, CASCADE_HASHING_L2));
      }
      else
      if (sNearestMatchingMethod == "FASTCASCADEHASHINGL2")
      {
        std::cout << "Using FAST_CASCADE_HASHING_L2 matcher" << std::endl;
        collectionMatcher.reset(new Cascade_Hashing_Matcher_Regions(fDistRatio));
      }
      if (!collectionMatcher)
      {
        std::cerr << "Invalid Nearest Neighbor method: " << sNearestMatchingMethod << std::endl;
      }
      // Perform the matching
      system::Timer timer;
      {
        // From matching mode compute the pair list that have to be matched:
        Pair_Set pairs;
        switch (ePairmode)
        {
          case PAIR_EXHAUSTIVE: pairs = exhaustivePairs(sfm_data.GetViews().size()); break;
          case PAIR_CONTIGUOUS: pairs = contiguousWithOverlap(sfm_data.GetViews().size(), iMatchingVideoMode); break;
          case PAIR_FROM_FILE:
            if (!loadPairs(sfm_data.GetViews().size(), sPredefinedPairList, pairs))
            {
            }
            break;
        }
        // Photometric matching of putative pairs
        collectionMatcher->Match(regions_provider, pairs, map_PutativesMatches, &progress);
        //---------------------------------------
        //-- Export putative matches
        //---------------------------------------
        if (!Save(map_PutativesMatches, std::string(sMatchesDirectory + "/matches.putative.bin")))
        {
          std::cerr
            << "Cannot save computed matches in: "
            << std::string(sMatchesDirectory + "/matches.putative.bin");
        }
      }
      std::cout << "Task (Regions Matching) done in (s): " << timer.elapsed() << std::endl;
    }
    //-- export putative matches Adjacency matrix
    PairWiseMatchingToAdjacencyMatrixSVG(vec_fileNames.size(),
      map_PutativesMatches,
      stlplus::create_filespec(sMatchesDirectory, "PutativeAdjacencyMatrix", "svg"));
    //-- export view pair graph once putative graph matches have been computed
    {
      std::set<IndexT> set_ViewIds;
      std::transform(sfm_data.GetViews().begin(), sfm_data.GetViews().end(),
        std::inserter(set_ViewIds, set_ViewIds.begin()), stl::RetrieveKey());
      graph::indexedGraph putativeGraph(set_ViewIds, getPairs(map_PutativesMatches));
      graph::exportToGraphvizData(
        stlplus::create_filespec(sMatchesDirectory, "putative_matches"),
        putativeGraph);
    }

    //---------------------------------------
    // b. Geometric filtering of putative matches
    //    - AContrario Estimation of the desired geometric model
    //    - Use an upper bound for the a contrario estimated threshold
    //---------------------------------------

    std::unique_ptr<ImageCollectionGeometricFilter> filter_ptr(
      new ImageCollectionGeometricFilter(&sfm_data, regions_provider));

    if (filter_ptr)
    {
      system::Timer timer;
      const double d_distance_ratio = 0.6;

      PairWiseMatches map_GeometricMatches;
      switch (eGeometricModelToCompute)
      {
        case HOMOGRAPHY_MATRIX:
        {
          const bool bGeometric_only_guided_matching = true;
          filter_ptr->Robust_model_estimation(
            GeometricFilter_HMatrix_AC(4.0, imax_iteration),
            map_PutativesMatches, bGuided_matching,
            bGeometric_only_guided_matching ? -1.0 : d_distance_ratio, &progress);
          map_GeometricMatches = filter_ptr->Get_geometric_matches();
        }
        break;
        case FUNDAMENTAL_MATRIX:
        {
          filter_ptr->Robust_model_estimation(
            GeometricFilter_FMatrix_AC(4.0, imax_iteration),
            map_PutativesMatches, bGuided_matching, d_distance_ratio, &progress);
          map_GeometricMatches = filter_ptr->Get_geometric_matches();
        }
        break;
        case ESSENTIAL_MATRIX:
        {
          filter_ptr->Robust_model_estimation(
            GeometricFilter_EMatrix_AC(4.0, imax_iteration),
            map_PutativesMatches, bGuided_matching, d_distance_ratio, &progress);
          map_GeometricMatches = filter_ptr->Get_geometric_matches();

          //-- Perform an additional check to remove pairs with poor overlap
          std::vector<PairWiseMatches::key_type> vec_toRemove;
          for (const auto & pairwisematches_it : map_GeometricMatches)
          {
            const size_t putativePhotometricCount = map_PutativesMatches.find(pairwisematches_it.first)->second.size();
            const size_t putativeGeometricCount = pairwisematches_it.second.size();
            const float ratio = putativeGeometricCount / static_cast<float>(putativePhotometricCount);
            if (putativeGeometricCount < 50 || ratio < .3f)  {
              // the pair will be removed
              vec_toRemove.push_back(pairwisematches_it.first);
            }
          }
          //-- remove discarded pairs
          for (const auto & pair_to_remove_it : vec_toRemove)
          {
            map_GeometricMatches.erase(pair_to_remove_it);
          }
        }
        break;
        case ESSENTIAL_MATRIX_ANGULAR:
        {
          filter_ptr->Robust_model_estimation(
            GeometricFilter_ESphericalMatrix_AC_Angular<false>(4.0, imax_iteration),
            map_PutativesMatches, bGuided_matching, d_distance_ratio, &progress);
          map_GeometricMatches = filter_ptr->Get_geometric_matches();
        }
        break;
        case ESSENTIAL_MATRIX_ORTHO:
        {
          filter_ptr->Robust_model_estimation(
            GeometricFilter_EOMatrix_RA(2.0, imax_iteration),
            map_PutativesMatches, bGuided_matching, d_distance_ratio, &progress);
          map_GeometricMatches = filter_ptr->Get_geometric_matches();
        }
        break;
        case ESSENTIAL_MATRIX_UPRIGHT:
        {
          filter_ptr->Robust_model_estimation(
            GeometricFilter_ESphericalMatrix_AC_Angular<true>(4.0, imax_iteration),
              map_PutativesMatches, bGuided_matching, d_distance_ratio, &progress);
          map_GeometricMatches = filter_ptr->Get_geometric_matches();
        }
        break;
      }

      //---------------------------------------
      //-- Export geometric filtered matches
      //---------------------------------------
      if (!Save(map_GeometricMatches,
        std::string(sMatchesDirectory + "/" + sGeometricMatchesFilename)))
      {
        std::cerr
            << "Cannot save computed matches in: "
            << std::string(sMatchesDirectory + "/" + sGeometricMatchesFilename);
      }

      std::cout << "Task done in (s): " << timer.elapsed() << std::endl;

      // -- export Geometric View Graph statistics
      graph::getGraphStatistics(sfm_data.GetViews().size(), getPairs(map_GeometricMatches));

      //-- export Adjacency matrix
      std::cout << "\n Export Adjacency Matrix of the pairwise's geometric matches"
        << std::endl;
      PairWiseMatchingToAdjacencyMatrixSVG(vec_fileNames.size(),
        map_GeometricMatches,
        stlplus::create_filespec(sMatchesDirectory, "GeometricAdjacencyMatrix", "svg"));

      //-- export view pair graph once geometric filter have been done
      {
        std::set<IndexT> set_ViewIds;
        std::transform(sfm_data.GetViews().begin(), sfm_data.GetViews().end(),
          std::inserter(set_ViewIds, set_ViewIds.begin()), stl::RetrieveKey());
        graph::indexedGraph putativeGraph(set_ViewIds, getPairs(map_GeometricMatches));
        graph::exportToGraphvizData(
          stlplus::create_filespec(sMatchesDirectory, "geometric_matches"),
          putativeGraph);
      }
    }
}

/// From 2 given image file-names, find the two corresponding index in the View list
bool computeIndexFromImageNames(
  const SfM_Data & sfm_data,
  const std::pair<std::string,std::string>& initialPairName,
  Pair& initialPairIndex)
{
  if (initialPairName.first == initialPairName.second)
  {
    std::cerr << "\nInvalid image names. You cannot use the same image to initialize a pair." << std::endl;
    return false;
  }

  initialPairIndex = {UndefinedIndexT, UndefinedIndexT};

  /// List views filenames and find the one that correspond to the user ones:
  for (Views::const_iterator it = sfm_data.GetViews().begin();
    it != sfm_data.GetViews().end(); ++it)
  {
    const View * v = it->second.get();
    const std::string filename = stlplus::filename_part(v->s_Img_path);
    if (filename == initialPairName.first)
    {
      initialPairIndex.first = v->id_view;
    }
    else{
      if (filename == initialPairName.second)
      {
        initialPairIndex.second = v->id_view;
      }
    }
  }
  return (initialPairIndex.first != UndefinedIndexT &&
      initialPairIndex.second != UndefinedIndexT);
}

void Dialog::on_pushButton_5_clicked()
{
    using namespace std;
    std::cout << "Sequential/Incremental reconstruction" << std::endl
              << " Perform incremental SfM (Initial Pair Essential + Resection)." << std::endl
              << std::endl;

    CmdLine cmd;

    std::string sSfM_Data_Filename;
    std::string sMatchesDir, sMatchFilename;
    std::string sOutDir = "";
    std::pair<std::string,std::string> initialPairString("","");
    std::string sIntrinsic_refinement_options = "ADJUST_ALL";
    int i_User_camera_model = PINHOLE_CAMERA_RADIAL3;
    bool b_use_motion_priors = false;
    int triangulation_method = static_cast<int>(ETriangulationMethod::DEFAULT);
    int resection_method  = static_cast<int>(resection::SolverType::DEFAULT);

    cmd.add( make_option('i', sSfM_Data_Filename, "input_file") );
    cmd.add( make_option('m', sMatchesDir, "matchdir") );
    cmd.add( make_option('M', sMatchFilename, "match_file") );
    cmd.add( make_option('o', sOutDir, "outdir") );
    cmd.add( make_option('a', initialPairString.first, "initialPairA") );
    cmd.add( make_option('b', initialPairString.second, "initialPairB") );
    cmd.add( make_option('c', i_User_camera_model, "camera_model") );
    cmd.add( make_option('f', sIntrinsic_refinement_options, "refineIntrinsics") );
    cmd.add( make_switch('P', "prior_usage") );
    cmd.add( make_option('t', triangulation_method, "triangulation_method"));
    cmd.add( make_option('r', resection_method, "resection_method"));

    if ( !isValid(static_cast<ETriangulationMethod>(triangulation_method))) {
      std::cerr << "\n Invalid triangulation method" << std::endl;
    }

    if ( !isValid(openMVG::cameras::EINTRINSIC(i_User_camera_model)) )  {
      std::cerr << "\n Invalid camera type" << std::endl;
    }

    const cameras::Intrinsic_Parameter_Type intrinsic_refinement_options =
      cameras::StringTo_Intrinsic_Parameter_Type(sIntrinsic_refinement_options);
    if (intrinsic_refinement_options == static_cast<cameras::Intrinsic_Parameter_Type>(0) )
    {
      std::cerr << "Invalid input for Bundle Adjusment Intrinsic parameter refinement option" << std::endl;
    }

    // Load input SfM_Data scene
    SfM_Data sfm_data;
    if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS|INTRINSICS))) {
      std::cerr << std::endl
        << "The input SfM_Data file \""<< sSfM_Data_Filename << "\" cannot be read." << std::endl;
    }

    // Init the regions_type from the image describer file (used for image regions extraction)
    using namespace openMVG::features;
    const std::string sImage_describer = stlplus::create_filespec(sMatchesDir, "image_describer", "json");
    std::unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);
    if (!regions_type)
    {
      std::cerr << "Invalid: "
        << sImage_describer << " regions type file." << std::endl;
    }

    // Features reading
    std::shared_ptr<Features_Provider> feats_provider = std::make_shared<Features_Provider>();
    if (!feats_provider->load(sfm_data, sMatchesDir, regions_type)) {
      std::cerr << std::endl
        << "Invalid features." << std::endl;
    }
    // Matches reading
    std::shared_ptr<Matches_Provider> matches_provider = std::make_shared<Matches_Provider>();
    if // Try to read the provided match filename or the default one (matches.f.txt/bin)
    (
      !(matches_provider->load(sfm_data, sMatchFilename) ||
        matches_provider->load(sfm_data, stlplus::create_filespec(sMatchesDir, "matches.f.txt")) ||
        matches_provider->load(sfm_data, stlplus::create_filespec(sMatchesDir, "matches.f.bin")))
    )
    {
      std::cerr << std::endl
        << "Invalid matches file." << std::endl;
    }

    if (sOutDir.empty())  {
      std::cerr << "\nIt is an invalid output directory" << std::endl;
    }

    if (!stlplus::folder_exists(sOutDir))
    {
      if (!stlplus::folder_create(sOutDir))
      {
        std::cerr << "\nCannot create the output directory" << std::endl;
      }
    }

    //---------------------------------------
    // Sequential reconstruction process
    //---------------------------------------

    openMVG::system::Timer timer;
    SequentialSfMReconstructionEngine sfmEngine(
      sfm_data,
      sOutDir,
      stlplus::create_filespec(sOutDir, "Reconstruction_Report.html"));

    // Configure the features_provider & the matches_provider
    sfmEngine.SetFeaturesProvider(feats_provider.get());
    sfmEngine.SetMatchesProvider(matches_provider.get());

    // Configure reconstruction parameters
    sfmEngine.Set_Intrinsics_Refinement_Type(intrinsic_refinement_options);
    sfmEngine.SetUnknownCameraType(EINTRINSIC(i_User_camera_model));
    b_use_motion_priors = cmd.used('P');
    sfmEngine.Set_Use_Motion_Prior(b_use_motion_priors);
    sfmEngine.SetTriangulationMethod(static_cast<ETriangulationMethod>(triangulation_method));
    sfmEngine.SetResectionMethod(static_cast<resection::SolverType>(resection_method));

    // Handle Initial pair parameter
    if (!initialPairString.first.empty() && !initialPairString.second.empty())
    {
      Pair initialPairIndex;
      if (!computeIndexFromImageNames(sfm_data, initialPairString, initialPairIndex))
      {
          std::cerr << "Could not find the initial pairs <" << initialPairString.first
            <<  ", " << initialPairString.second << ">!\n";
      }
      sfmEngine.setInitialPair(initialPairIndex);
    }

    if (sfmEngine.Process())
    {
      std::cout << std::endl << " Total Ac-Sfm took (s): " << timer.elapsed() << std::endl;

      std::cout << "...Generating SfM_Report.html" << std::endl;
      Generate_SfM_Report(sfmEngine.Get_SfM_Data(),
        stlplus::create_filespec(sOutDir, "SfMReconstruction_Report.html"));

      //-- Export to disk computed scene (data & visualizable results)
      std::cout << "...Export SfM_Data to disk." << std::endl;
      Save(sfmEngine.Get_SfM_Data(),
        stlplus::create_filespec(sOutDir, "sfm_data", ".bin"),
        ESfM_Data(ALL));

      Save(sfmEngine.Get_SfM_Data(),
        stlplus::create_filespec(sOutDir, "cloud_and_poses", ".ply"),
        ESfM_Data(ALL));

    }

}
