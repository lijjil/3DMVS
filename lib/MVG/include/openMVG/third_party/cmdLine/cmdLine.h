/**
 * @file cmdLine.h
 * @brief Command line option parsing
 * @author Pascal Monasse
 *
 * Copyright (c) 2012-2014 Pascal Monasse
 * All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CMDLINE_H
#define CMDLINE_H

#include <vector>
#include <algorithm>
#include <iostream>
#include <string>
#include <sstream>
#include <cassert>

#ifdef _WIN32
#pragma warning(disable:4290)       // exception specification ignored...
#endif

/// 类中包含3个数据成员，该类为父类，有两个子类（后面介绍）
/// Base class for option/switch
class Option {
public:
    char c; ///< Option letter (eg 's' for option -s)   ///短标识符:i,d,o等
    bool used; ///< Does the command line use that option?  ///用于说明该参数是否已经被检测到，在解析输入参数时使用
    std::string longName; /// Optional long name (eg "switch" for --switch) /// 长名标识符:imageDirectory等

    /// Constructor with short name/long name   //构造函数
    Option(char d, std::string name)
    : c(d), used(false), longName(name) {}
    virtual ~Option() = default;
    virtual bool check(int& argc, char* argv[])=0; ///< Option found at argv[0]?   ///检查是否存在有对应的符合格式的参数存在，并解析，具体解析见子类
    virtual Option* clone() const=0; ///< Copy
};

/// Option on/off is called a switch
class OptionSwitch : public Option {
public:
    /// Constructor with short name/long name (optional)
    OptionSwitch(char c, std::string name="")
    : Option(c,name) {}
    /// Find switch in argv[0]
    bool check(int& argc, char* argv[]) override {
        if(std::string("-")+c==argv[0] ||
           (!longName.empty() && std::string("--")+longName==argv[0])) {
            used = true;
            std::rotate(argv, argv+1, argv+argc);
            argc -= 1;
            return true;
        } else if(std::string(argv[0]).find(std::string("-")+c)==0) {
            used = true; // Handle multiple switches in single option
            std::rotate(argv[0]+1, argv[0]+2,
                        argv[0]+std::string(argv[0]).size()+1);
            return true;
        }
        return false;
    }
    /// Copy
    Option* clone() const override {
        return new OptionSwitch(c, longName);
    }
};



/// Option with an argument of type T, which must be readable by operator>>
template <class T>
class OptionField : public Option {
public:
    //构造函数c,name分别对应父类Option种的c,longName;field对应参数比如例子中的输入参数images/sensor_width_camera_database.txt等
    //因为输入参数类型不固定，可能是字符串也可能是数字，所以采用模板类
    /// Constructor. The result with be stored in variable @field.
    OptionField(char c, T& field, std::string name="")
    : Option(c,name), _field(field) {}
    /// 查找正确的输入参数
    /// Find option in argv[0] and argument in argv[1]. Throw an exception
    /// (type std::string) if the argument cannot be read.
    bool check(int& argc, char* argv[]) override {      //这里的argc，argv形参，并不与主程序中的argc，argv等同
        std::string param; int arg=0;
        if(std::string("-")+c==argv[0] ||
           (!longName.empty() && std::string("--")+longName==argv[0])) {    // 对应输入参数格式为：-i ImageDataset_SceauxCastle/images
            if(argc<=1)                                                        // 或：--imageDirectory ImageDataset_SceauxCastle/images
                throw std::string("Option ")
                    +argv[0]+" requires argument";
            param=argv[1]; arg=2;                   // 将ImageDataset_SceauxCastle/images赋值給param
        } else if(std::string(argv[0]).find(std::string("-")+c)==0) {     // 对应输入参数格式为：-iImageDataset_SceauxCastle/images
            param=argv[0]+2; arg=1;                 // 将ImageDataset_SceauxCastle/images赋值給param
        } else if(!longName.empty() &&              // 对应输入参数格式为：--imageDirectory=ImageDataset_SceauxCastle/images
                  std::string(argv[0]).find(std::string("--")+longName+'=')==0){
            size_t size=(std::string("--")+longName+'=').size();
            param=std::string(argv[0]).substr(size); arg=1;     //将ImageDataset_SceauxCastle/images赋值給param
        }
        if(arg>0) {     //arg>0代表检查到了符合格式的输入参数
            if(! read_param(param))     //读入检测到的参数，如果不成功，抛出异常
                throw std::string("Unable to interpret ")
                    +param+" as argument of "+argv[0];
            used = true;        //该option对象对应的参数已经检测到
            std::rotate(argv, argv+arg, argv+argc); //把从参数argv到argv + argc之间的参数序列看成一个圆，对他们进行旋转，旋转后的圆的第一个元素为argv + arg
                                                                //作用就是将检测到的参数放在输入参数序列后面
                                                                //rotate函数的用法详细介绍见：http://c.biancheng.net/view/609.html
                                                                /*
            比如原始序列为：openMVG_main_SfMInit_ImageListing -i images -d images/sensor_width_camera_database.txt -o matches
            假如argv对应-i  argv + argc对应images/sensor_width_camera_database.txt   arg=2
            -i images -d images/sensor_width_camera_database.txt构成待旋转的圆，argv + arg对应的-d为旋转后圆的第一个元素
            那么rotate后的新序列为openMVG_main_SfMInit_ImageListing -d images/sensor_width_camera_database.txt -i images -o matches
            */
            argc -= arg;
            return true;
        }
        return false;
    }
    /// 将读到的参数param赋值给_field；param为string型，_field类型在OptionField对象中预先定义了，见于make_option函数，也在cmdLine.h文件中
    /// Decode the string as template type T
    bool read_param(const std::string& param) {
        std::stringstream str(param); char unused;
        return !((str >> _field).fail() || !(str>>unused).fail());
    }
    /// Copy
    Option* clone() const override {
        return new OptionField<T>(c, _field, longName);
    }
private:
    T& _field; ///< Reference to variable where to store the value
};

/// Template specialization to be able to take parameter including space.
/// Generic method would do >>_field (stops at space) and signal unused chars.
template <>
inline bool OptionField<std::string>::read_param(const std::string& param) {
    _field = param;
    return true;
}

/// New switch option
OptionSwitch make_switch(char c, std::string name="") {
    return OptionSwitch(c, name);
}

/// New option with argument.
template <class T>
OptionField<T> make_option(char c, T& field, std::string name="") {
    return OptionField<T>(c, field, name);
}

/// Command line parsing
class CmdLine {
    std::vector<Option*> opts;      //opts中包含所有的必须的或者可选的参数对应的标识符；即该程序需要输入哪些参数
public:
    /// Destructor
    ~CmdLine() {
        std::vector<Option*>::iterator it=opts.begin();
        for(; it != opts.end(); ++it)
            delete *it;
    }
    /// Add an option           //在Option对象向量中添加Option对象
    void add(const Option& opt) {
        opts.push_back( opt.clone() );
    }


    /// ************主要的成员函数************
    /// 实现输入参数与程序需求的参数进行匹配
    /// Parse of command line acting as a filter. All options are virtually
    /// removed from the command line.
    void process(int& argc, char* argv[]) {
        std::vector<Option*>::iterator it=opts.begin();
        for(; it != opts.end(); ++it)
            (*it)->used = false;
        for(int i=1; i<argc;) {
            // "--" 为参数结尾标志，不知道这个有什么用
            if(std::string("--")==argv[i]) { // "--" means stop option parsing
                std::rotate(argv+i, argv+i+1, argv+argc);   //旋转，使"--"为序列结尾
                -- argc;
                break;
            }
            bool found=false; // Find option
            for(it=opts.begin(); it != opts.end(); ++it) {      //将预先定义的Option类的各个对象分别与输入参数进行匹配
                int n = argc-i;
                found = (*it)->check(n, argv+i);        //argv + i(argv[1])对应的参数与当前Option对象匹配
                if(found) {
                    argc = n+i;             //这行也不知道有什么用
                    break;
                }
            }
            //如果没找到合适的匹配，但是输入的参数开头为"_"，则抛出异常未识别标识符
            if(! found) { // A negative number is not an option
                if(std::string(argv[i]).size()>1 && argv[i][0] == '-') {
                    std::istringstream str(argv[i]);
                    float v;
                    if(! (str>>v).eof())
                        throw std::string("Unrecognized option ")+argv[i];
                }
                ++i;
            }
        }
    }
    /// Was the option used in last parsing?
    bool used(char c) const {
        std::vector<Option*>::const_iterator it=opts.begin();
        for(; it != opts.end(); ++it)
            if((*it)->c == c)
                return (*it)->used;
        assert(false); // Called with non-existent option, probably a bug
        return false;
    }
};

#endif
