һ��Windowsƽ̨�¿�ִ���ļ�����ʵ��
1. �ڸ�Ŀ¼�µ�source�ļ����ڱ�����VC8��VC9�����汾�Ľ��������˫�����ɴ򿪡�
2. ��Ŀ���ɵĿ�ִ���ļ�*.exe��λ��sourceĿ¼�µ�bin�ļ����ڡ�


����Windowsƽ̨��CMake������������Ĺ���
Windows�û���Ҫ��װCMake�������ߣ�����������ص�ַ��https://cmake.org/download/
�ڰ�װ��������Ҫѡ�񻷾�����·����ϵͳall users��ӵ�ѡ�
�ڰ�װ��ɺ����黷������path·���У��Ƿ�CMake·���Ѿ���ӡ�
ǿ�ҽ���CMake��װ��ע�����������������һ�Ρ�
VC�汾����Visual Studio�汾�Ŷ�Ӧ��ϵ
MSVC++ 14.0 _MSC_VER == 1900 (Visual Studio 2015)
MSVC++ 12.0 _MSC_VER == 1800 (Visual Studio 2013)
MSVC++ 11.0 _MSC_VER == 1700 (Visual Studio 2012)
MSVC++ 10.0 _MSC_VER == 1600 (Visual Studio 2010)
MSVC++ 9.0  _MSC_VER == 1500 (Visual Studio 2008)
MSVC++ 8.0  _MSC_VER == 1400 (Visual Studio 2005)

����Windowsƽ̨�¿�ִ���ļ�����ʵ��
����AVS��׼�齨��ο������������*.bat�ļ���������ǵ�*.bat�ļ�ȫ����׺Ϊ*.txt��
�򵥵أ�CMake���������������ʹ�÷������£�
1. ��buildĿ¼�µ�Rename-all.bat.txt�ļ��ֶ�������ΪRename-all.bat��ִ��Rename-all.bat�ļ�����������������*.bat.txt������Ϊ*.bat��
2. ��buildĿ¼��ѡ���ʺ��Լ���Visual Studio����ƽ̨�ļ��У���vc12-x86_64(Visual Studio 2013 & X64)��˫��ִ��build-all.bat�ļ���
3. RD.sln��CMake���ɵĽ���������򿪲��������ɿ�ִ���ļ�����Ŀ���ɵĿ�ִ���ļ�*.exe��λ��sourceĿ¼�µ�bin�ļ����ڡ�
4. ��ִ��clean.bat��������CMake������Visual Studio���������ͬʱҲ��ɾ��bin�µĿ�ִ���ļ�����bin�ļ��м���ص������ļ����ᱻɾ����


�ġ�Linuxƽ̨�¿�ִ���ļ�����ʵ��
1. ����build/linux�ļ���
2. ִ��make-all.sh�ļ����ɿ�ִ���ļ���
3. ִ��clean.sh�ļ�ɾ����ִ���ļ���

---------------------------------------------------
����	guojiang_sc@126.com
������	yiminzhou@uestc.edu.cn
�������ʣ���ӭ��λ��ϵָ����