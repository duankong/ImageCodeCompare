一、Windows平台下可执行文件生成实例
1. 在根目录下的source文件夹内保留了VC8和VC9两个版本的解决方案，双击即可打开。
2. 项目生成的可执行文件*.exe将位于source目录下的bin文件夹内。


二、Windows平台下CMake创建解决方案的过程
Windows用户需要安装CMake构建工具，官网免费下载地址：https://cmake.org/download/
在安装过程中需要选择环境变量路径向系统all users添加的选项。
在安装完成后，请检查环境变量path路径中，是否CMake路径已经添加。
强烈建议CMake安装后，注销或重新启动计算机一次。
VC版本号与Visual Studio版本号对应关系
MSVC++ 14.0 _MSC_VER == 1900 (Visual Studio 2015)
MSVC++ 12.0 _MSC_VER == 1800 (Visual Studio 2013)
MSVC++ 11.0 _MSC_VER == 1700 (Visual Studio 2012)
MSVC++ 10.0 _MSC_VER == 1600 (Visual Studio 2010)
MSVC++ 9.0  _MSC_VER == 1500 (Visual Studio 2008)
MSVC++ 8.0  _MSC_VER == 1400 (Visual Studio 2005)

三、Windows平台下可执行文件生成实例
由于AVS标准组建议参考软件发布不带*.bat文件，因此我们的*.bat文件全部后缀为*.txt。
简单地，CMake构建解决方案具体使用方法如下：
1. 将build目录下的Rename-all.bat.txt文件手动重命名为Rename-all.bat并执行Rename-all.bat文件。它将把其余所有*.bat.txt重命名为*.bat。
2. 在build目录下选择适合自己的Visual Studio开发平台文件夹，如vc12-x86_64(Visual Studio 2013 & X64)，双击执行build-all.bat文件。
3. RD.sln是CMake生成的解决方案，打开并编译生成可执行文件。项目生成的可执行文件*.exe将位于source目录下的bin文件夹内。
4. 若执行clean.bat用于清理CMake构建的Visual Studio解决方案，同时也会删除bin下的可执行文件。但bin文件夹及相关的配置文件不会被删除。


四、Linux平台下可执行文件生成实例
1. 进入build/linux文件。
2. 执行make-all.sh文件生成可执行文件。
3. 执行clean.sh文件删除可执行文件。

---------------------------------------------------
郭江	guojiang_sc@126.com
周益民	yiminzhou@uestc.edu.cn
如有疑问，欢迎各位联系指正！