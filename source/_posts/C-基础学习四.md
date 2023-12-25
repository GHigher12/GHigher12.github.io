---
title: C++基础学习四
typora-root-url: ..\imgs
date: 2022-10-22 18:35:09
tags: C/C++
categories: 
        - 程序设计
        - C/C++
---

# C++基础学习四

## 文本文件

### 写文件

**步骤：**

- 包含头文件:`#include <fstream>`
- 创建流对象:`ofstream ofs;`
- 打开文件:`ofs.open("文件路径", 打开方式);`
- 写数据:`ofs<<"写入数据";`
- 关闭文件:`ofs.close;`

|  打开方式   |            解释            |
| :---------: | :------------------------: |
|   ios::in   |     为读文件而打开文件     |
|  ios::out   |     为写文件而打开文件     |
|  ios::ate   |      初始位置:文件尾       |
|  ios::app   |       追加方式写文件       |
| ios::trunc  | 如果文件存在先删除，再创建 |
| ios::binary |         二进制方式         |

```C++
#include <iostream>
#include <fstream>
using namespace std;
void test01()
{
    ofstream ofs;
    ofs.open("D:\\C++_CLion\\Demo01\\FILE\\test.txt", ios::out);
    ofs<<"姓名:张三"<<endl;
    ofs<<"年龄:18"<<endl;
    ofs<<"性别:男"<<endl;
    ofs.close();
}
int main()
{
    test01();
    return 0;
}
```

![image-20221022193600777](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20221022193600777.png)

### 读文件

**步骤：**

- 包含头文件:`#include <fstream>`
- 创建流对象:`ofstream ifs;`
- 打开文件:`ifs.open("文件路径", 打开方式);`
- 读数据:四种方式读数据;`
- 关闭文件:`ifs.close;`

```cpp
void test02()
{
    ofstream ifs;
    ifs.open("D:\\C++_CLion\\Demo01\\FILE\\test.txt", ios::in);
//    int a;
    if(!ifs.is_open())
    {
        cout<<"文件打开失败"<<endl;
        return;
    }

    char buf[1024] = {0};
//    cin>>a;
    while (ifs >> buf)
    {
        cout<<buf<<endl;
    }
//    char buf[1024] = {0};
//    string buf;
//    while (getline(ifs, buf))
//    {
//        cout<<buf<<endl;
//    }
    ifs.close();
}
int main()
{
    test02();
    return 0;
}
```

***CLion中读写文件使用绝对路径才可以，不知道是什么原因，等后续找到解决方法再更新。。。***

## 职工管理系统

### 管理系统需求

公司中职工分为三类:普通员工、经理、老板，显示信息时，需要显示职工编号、职工姓名、职工岗位、以及职责

- 普通员工职责:完成经理交给的任务
- 经理职责:完成老板交给的任务，并下发任务给员工
- 老板职责:管理公司所有事务

**管理系统中需要实现的功能如下:**

- 退出管理程序:退出当前管理系统
- 增加职工信息:实现批量添加职工功能，将信息录入到文件中，职工信息为:职工编号、姓名、部门编号
- 显示职工信息:显示公司内部所有职工的信息
- 删除离职职工:按照编号删除指定的职工·修改职工信息:按照编号修改职工个人信息
- 查找职工信息:按照职工的编号或者职工的姓名进行查找相关的人员信息·按照编号排序:按照职工编号，进行排序，排序规则由用户指定·
- 清空所有文档:清空文件中记录的所有职工信息(清空前需要再次确认，防止误删)

### 创建管理类

创建`workermanager.cpp`和`workermanager.h`文件

`workermanager.h`

```cpp
#ifndef DEMO01_WORKERMANGER_H
#define DEMO01_WORKERMANGER_H
#include <iostream>
using namespace std;
class WorkerManager
{
public:
    WorkerManager();
    ~WorkerManager();
};
#endif //DEMO01_WORKERMANGER_H
```

`workermanager.cpp`

```Cpp
#include "workermanger.h"

WorkerManager::WorkerManager() {

}
WorkerManager::~WorkerManager() {
    
}
```

### 菜单功能

`workermanager.h`

```cpp
class WorkerManager
{
public:
    WorkerManager();
    void Show_Menu();
    ~WorkerManager();
};
```

`workermanager.cpp`

```cpp
void WorkerManager::Show_Menu()
{
    cout<<"************************"<<endl;
    cout<<"****欢迎使用职工管理系统***"<<endl;
    cout<<"**** 1、增加职工信息 ****"<<endl;
    cout<<"**** 2、显示职工信息 ****"<<endl;
    cout<<"**** 3、删除离职职工 ****"<<endl;
    cout<<"**** 4、修改职工信息 ****"<<endl;
    cout<<"**** 5、查找职工信息 ****"<<endl;
    cout<<"**** 6、按照编号排序 ****"<<endl;
    cout<<"**** 7、清空管理文档 ****"<<endl;
    cout<<"**** 0、退出管理程序 ****"<<endl;
    cout<<"************************"<<endl;
    cout<<endl;
}
```

新建`main_Company.cpp`包含`main`函数

```cpp
#include <iostream>
#include "workermanager.h"
using namespace std;

int main()
{
    WorkerManager wm;
    wm.Show_Menu();
    system("pause");
    return 0;
}
```

### 退出功能

`main_Company.cpp`

```cpp
int main()
{
    WorkerManager wm;
    int option = 0;

    while (true) {
        wm.ShowMenu();
        cout<<"请输入您的选择:"<<endl;
        cin>>option;
        switch (option)
        {
            case 0:
                wm.ExitSystem();
                break;
            case 1:
                break;
            case 2:
                break;
            case 3:
                break;
            case 4:
                break;
            case 5:
                break;
            case 6:
                break;
            case 7:
                break;
            default:
                system("cls");
                break;
        }
    }
    system("pause");
    return 0;
}
```

`workermanager.cpp`

```cpp
void WorkerManager::ExitSystem()
{
    cout<<"欢迎下次使用...."<<endl;
//    system("pause");
    exit(0);
}
```

### 创建职工类

#### 创建父类职工

用多态方法编写`worker.h`，纯虚函数编写父类，无实例化对象

```cpp
#ifndef DEMO01_WORKER_H
#define DEMO01_WORKER_H
#include <iostream>
using namespace std;
#include <string>
class Worker
{
public:
    virtual void ShowInfo() = 0; //纯虚函数
    virtual string getDeptName() = 0;

    int m_Id;
    string m_Name;
    int m_DeptId;
};
#endif 
```

#### 编写员工类

`employee.h`

```cpp
#ifndef DEMO01_EMPLOYEE_H
#define DEMO01_EMPLOYEE_H
#include <iostream>
using namespace std;
#include "worker.h"

class Employee:public Worker //继承
{
public:
    Employee(int id, string name, int did);
    virtual void ShowInfo(); //子类重写虚构函数
    virtual string getDeptName();
};

#endif 
```

·`employee.cpp`

```cpp
#include "employee.h"

Employee::Employee(int id, string name, int did)
{
    this->m_Id = id;  // this指针 指向被调用的成员函数 所属的对象---Worker
    this->m_Name = name;
    this->m_DeptId = did;
}
void Employee::ShowInfo()
{
    cout<<"职工编号: "<<this->m_Id
    <<"\t职工姓名: "<<this->m_Name
    <<"\t岗位: "<<this->getDeptName()
    <<"\t岗位名称:完成经理交给的任务"<<endl;
}
string Employee::getDeptName()
{
    return string("员工");
}
```

#### 编写经理类

`manager.h`

```cpp
#ifndef DEMO01_MANAGER_H
#define DEMO01_MANAGER_H
#include "iostream"
#include "worker.h"
using namespace std;
class Manager:public Worker
{
public:
    Manager(int id, string name, int did);
    virtual void ShowInfo();
    virtual string getDeptName();
};
#endif
```

`manager.cpp`

```cpp
#include "manager.h"

Manager::Manager(int id, string name, int did)
{
    this->m_Id = id;
    this->m_Name = name;
    this->m_DeptId = did;
}
void Manager::ShowInfo()
{
    cout<<"职工编号: "<<this->m_Id
    <<"\t职工姓名: "<<this->m_Name
    <<"\t岗位: "<<this->getDeptName()
    <<"\t岗位名称:完成老板交给的任务, 并发布给员工"<<endl;
}
string Manager::getDeptName()
{
    return string("经理");
}
```

#### 编写老板类

`boss.h`

```cpp
#ifndef DEMO01_BOSS_H
#define DEMO01_BOSS_H
#include <iostream>
using namespace std;
#include "worker.h"

class Boss:public Worker
{
public:
    Boss(int id, string name, int did);
    virtual void ShowInfo();
    virtual string getDeptName();
};
#endif
```

`boss.cpp`

```cpp
#include "boss.h"

Boss::Boss(int id, string name, int did)
{
    this->m_Id = id;
    this->m_Name = name;
    this->m_DeptId = did;
}
void Boss::ShowInfo()
{
    cout<<"职工编号: "<<this->m_Id
    <<"\t职工姓名: "<<this->m_Name
    <<"\t岗位: "<<this->getDeptName()
    <<"\t岗位名称:给经理发布任务"<<endl;
}
string Boss::getDeptName()
{
    return string("老板");
}
```

#### 测试

```cpp
#include <iostream>
#include "worker.h"
#include "employee.h"
#include "manager.h"
#include "boss.h"
using namespace std;

int main()
{
    Worker * worker = NULL;
    worker = new Employee(1, "张三", 1);
    worker->ShowInfo();
    delete worker;
    worker = new Manager(2, "李四", 2);
    worker->ShowInfo();
    delete worker;
    worker = new Boss(3, "王五", 3);
    worker->ShowInfo();
    delete worker;
}
//---------------结果------------------
职工编号: 1	职工姓名: 张三	岗位: 员工	岗位名称:完成经理交给的任务
职工编号: 2	职工姓名: 李四	岗位: 经理	岗位名称:完成老板交给的任务, 并发布给员工
职工编号: 3	职工姓名: 王五	岗位: 老板	岗位名称:给经理发布任务
```

### 添加职工

功能描述: 批星添加职工，并且保存到文件中

#### 功能分析

分析:用户在批量创建时，可能会创建不同种类的职工

如果想将所有不同种类的员工都放入到一个数组中，可以将所有员工的指针维护到一个数组里如果想在程序中维护这个不定长度的数组，可以将

数组创建到堆区，并利用`Worker ** `的指针维护

![image-20221215200510436](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20221215200510436.png)

#### 功能实现

`workermanager.cpp`

```cpp
void WorkerManager::AddEmp() {
    cout<<"请输入添加员工数量"<<endl;
    int addNum = 0;
    cin>>addNum;
    if(addNum>0)
    {
        //计算添加空间大小
        int newSize = this->m_EmpNum + addNum;
        //开辟新空间
        Worker ** newSpace = new Worker*[newSize];
        if(this->m_EmpArray != NULL)
        {
            for(int i=0; i<this->m_EmpNum; i++)
            {
                newSpace[i] = this->m_EmpArray[i];
            }
        }
        //批量添加新数据
        for(int i=0; i<addNum; i++)
        {
            int id;
            string name;
            int dSelet;
            cout<<"请输入第"<<i+1<<"个新职工编号"<<endl;
            cin>>id;
            cout<<"请输入第"<<i+1<<"个新职工姓名"<<endl;
            cin>>name;
            cout<<"请选择职工编号"<<endl;
            cout<<"1、普通员工"<<endl;
            cout<<"2、经理"<<endl;
            cout<<"3、老板"<<endl;
            cin>>dSelet;
            Worker * worker = NULL;
            switch(dSelet)
            {
                case 1:
                    worker = new Employee(id, name, 1);break;
                case 2:
                    worker = new Manager(id, name, 2);break;
                case 3:
                    worker = new Boss(id, name, 3);break
            }
            newSpace[this->m_EmpNum + i] = worker;
        }
        //释放原有的空间
        delete[] this->m_EmpArray;
        //更改新空间指向
        this->m_EmpArray = newSpace;
        //更新新的职工系数
        this->m_EmpNum = newSize;
        cout<<"成功添加"<<addNum<<"新职工"<<endl;

}
    else{
        cout<<"输入数据有误"<<endl;
    }
    system("pause");
    system("cls");
}
//堆区内存手动释放
WorkerManager::~WorkerManager() {
    if(this->m_EmpArray != NULL)
    {
        delete[] this->m_EmpArray;
        this->m_EmpArray = NULL;
    }
}   
```

`main_Company.cpp`主函数中添加职工函数`AddEmp()`

```cpp
int main()
{
    ......
    while (true) {
        wm.ShowMenu();
        cout<<"请输入您的选择:"<<endl;
        cin>>option;
        switch (option)
        {
            case 0:
                wm.ExitSystem();
                break;
            case 1:
                wm.AddEmp();
                break;
 		......
        }
    }
    }
```

### 文件交互-写文件                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           

在同目录下新建`EMP_FILE`文件夹

```cpp
#include <fstream>
#define FILENAME "D:\\C++_CLion\\Demo01\\Employee\\EMP_FILE\\empfile.txt"
```

```cpp
void WorkerManager::AddEmp() {
    ...
        //保存数据
        this->SaveFile();
	...
}
void WorkerManager::SaveFile()
{
    ofstream ofs;
    ofs.open(FILENAME, ios::out);
    for(int i=0; i<this->m_EmpNum; i++)
    {
        ofs<<this->m_EmpArray[i]->m_Id<<" "
        <<this->m_EmpArray[i]->m_Name<<" "
        <<this->m_EmpArray[i]->m_DeptId<<endl;
    }
    ofs.close();
}
```

### 文件交互-读文件

功能描述: 将文件中的内容读取到程序中

虽然我们实现了添加职工后保存到文件的操作，但是每次开始运行程序，并没有将文件中数据读取到程序中

而我们的程序功能中还有清空文件的需求

因此构造函数初始化数据的情况分为三种

> 1.第一次使用，文件未创建
>
> 2.文件存在，但是数据被用户清空
>
> 3.文件存在，并且保存职工的所有数据

`workermanager.h`

```cpp
class WorkerManager
{
public:
    WorkerManager();
    void ShowMenu();
    void ExitSystem();
    int m_EmpNum;
    void AddEmp();
    void SaveFile();
    //判断读文件是否为空
    bool m_FileIsEmpty;
    int GetEmpNum();
    void Init_Emp();
    Worker ** m_EmpArray;
    ~WorkerManager();
};
```

`workermanager.cpp`

```cpp
//构造函数
WorkerManager::WorkerManager() {
    ifstream ifs;
    ifs.open(FILENAME, ios::in);
    //第一次使用，文件未创建
    if(!ifs.is_open())
    {
        cout<<"文件不存在"<<endl;
        this->m_EmpNum = 0;
        this->m_EmpArray = NULL;
        this->m_FileIsEmpty = true;
        ifs.close();
        return;
    }
    //文件存在，但是数据被用户清空
    char ch;
    ifs>>ch;
    if(ifs.eof())
    {
        cout<<"文件为空"<<endl;
        this->m_EmpNum = 0;
        this->m_EmpArray = NULL;
        this->m_FileIsEmpty = true;
        ifs.close();
        return;
    }
    int num = this->GetEmpNum();
    cout<<"职工人数为:"<<num<<endl;
    this->m_EmpNum = num;
    //***********测试********
    this->m_EmpArray = new Worker * [this->m_EmpNum];
    this->Init_Emp();
    for(int i=0; i<this->m_EmpNum; i++)
    {
        cout<<"职工编号:"<<this->m_EmpArray[i]->m_Id
        <<" 姓名:"<<this->m_EmpArray[i]->m_Name
        <<" 部门编号:"<<this->m_EmpArray[i]->m_DeptId<<endl;
    }
}
void WorkerManager::AddEmp() {
    ...
        //保存数据
        this->m_FileIsEmpty = false;
	...
}
//获得员工数量
int WorkerManager::GetEmpNum()
{
    ifstream ifs;
    ifs.open(FILENAME, ios::in);//打开文件,读
    int id;
    string name;
    int dId;
    int num = 0;
    while(ifs>>id && ifs>>name && ifs>>dId)
    {
        num++;
    }
    return num;
}
//员工初始化
void WorkerManager::Init_Emp()
{
    ifstream ifs;
    ifs.open(FILENAME, ios::in);
    int id;
    string name;
    int dId;
    int index = 0;
    while(ifs>>id && ifs>>name && ifs>>dId)
    {
        Worker * worker = NULL;
        if(dId == 1)
        {
            worker = new Employee(id, name, dId);
        }
        if(dId == 2)
        {
            worker = new Manager(id, name, dId);
        }
        if(dId == 3)
        {
            worker = new Boss(id, name, dId);
        }
        this->m_EmpArray[index] = worker;
        index++;
    }
    ifs.close();
}
//***********结果************
职工人数为:2
职工编号:1 姓名:张三 部门编号:3
职工编号:2 姓名:李四 部门编号:3
......
```

### 显示职工

```cpp
void WorkerManager::ShowEmp()
{
    if(this->m_FileIsEmpty)
    {
        cout<<"文件不存在或者为空!!!"<<endl;
    }
    else
    {
        for(int i=0;i<m_EmpNum;i++)
        {
            this->m_EmpArray[i]->ShowInfo();
        }
        system("pause");
    }
}
```

```cpp
int main()
{
    ......
    while (true) {
        wm.ShowMenu();
        cout<<"请输入您的选择:"<<endl;
        cin>>option;
        switch (option)
        {
         ......
            case 2:
                wm.ShowEmp();
                break;
 		......
        }
    }
    }
```

### 删除职工

```cpp
void WorkerManager::DelEmp()
{
    if(this->m_FileIsEmpty)
    {
        cout<<"文件不存在或者为空!!!"<<endl;
    }
    else
    {
        cout<<"请输入想要删除的职工编号"<<endl;
        int id = 0;
        cin>>id;
        int index = this->IsExist(id);
        if(index != -1)
        {
            //数据迁移
            for(int i=index;i<this->m_EmpNum-1;i++)
            {
                this->m_EmpArray[i] = this->m_EmpArray[i+1];
            }
            this->m_EmpNum--;
            this->SaveFile();
            cout<<"删除成功"<<endl;
        }
        else
        {
            cout<<"删除失败,未找到职工"<<endl;
        }

    }
    system("pause");
}
int WorkerManager::IsExist(int id)
{
    int index = -1;
    for(int i=0;i<this->m_EmpNum;i++)
    {
        if(this->m_EmpArray[i]->m_Id == id)
        {
            index = i;
            break;
        }
    }
    return index;
}
```

### 修改职工

```cpp
void WorkerManager::ModEmp()
{
    if(this->m_FileIsEmpty)
    {
        cout<<"文件不存在或者为空!!!"<<endl;
    }
    else
    {
        cout<<"请输入修改职工的编号:"<<endl;
        int id;
        cin>>id;
        int ret = this->IsExist(id);
        if(ret != -1)
        {
            //查找到编号的职工
            delete this->m_EmpArray[ret];
            int newId = 0;
            string newName;
            int newdId;
            cout<<"查找到"<<id<<"号职工，请输入新职工号"<<endl;
            cin>>newId;
            cout<<"请输入新姓名"<<endl;
            cin>>newName;
            cout<<"请输入岗位"<<endl;
            cout<<"1、普通员工"<<endl;
            cout<<"2、经理"<<endl;
            cout<<"3、老板"<<endl;
            cin>>newdId;
            Worker * worker = NULL;
            switch(newdId)
            {
                case 1:
                    worker = new Employee(newId, newName, newdId);break;
                case 2:
                    worker = new Manager(newId, newName, newdId);break;
                case 3:
                    worker = new Boss(newId, newName, newdId);break;
            }
            this->m_EmpArray[ret] = worker;
            cout<<"修改成功"<<endl;
            this->SaveFile();
        }
        else
        {
            cout<<"修改失败,查无此人"<<endl;
        }
    }
    system("pause");
}
```

### 查找职工

```cpp
void WorkerManager::FindEmp()
{
    if(this->m_FileIsEmpty)
    {
        cout<<"文件不存在或者为空!!!"<<endl;
    }
    else
    {
        cout<<"请输入查找的方式"<<endl;
        cout<<"1、按照职工编号查找"<<endl;
        cout<<"2、按照职工姓名查找"<<endl;
        int select;
        cin>>select;
        if(select == 1)
        {
            int id;
            cout<<"请输入查找的职工编号"<<endl;
            cin >> id;
            int ret = IsExist(id);
            if(ret != -1)
            {
                //找到这个人
                cout<<"查找成功"<<endl;
                this->m_EmpArray[ret]->ShowInfo();
            }
            else
            {
                cout<<"查找失败,查无此人"<<endl;
            }
        }
        else if (select == 2)
        {
            bool flag = false;
            string name;
            cout<<"请输入查找的职工姓名"<<endl;
            cin >> name;
            for(int i=0;i<this->m_EmpNum;i++)
            {
             if(this->m_EmpArray[i]->m_Name == name)
             {
                 cout<<"查找成功, 职工编号为"<<this->m_EmpArray[i]->m_Id<<"职工信息如下:"<<endl;
                 flag = true;
                 this->m_EmpArray[i]->ShowInfo();
             }
            }
            if(flag == false)
            {
                cout<<"查找失败，查无此人"<<endl;
            }
        }
        else
        {
            cout<<"输入错误!!!"<<endl;
        }
    }
    system("pause");
}
```

### 职工排序

```cpp
void WorkerManager::SortEmp()
{
    if(this->m_FileIsEmpty)
    {
        cout<<"文件不存在或者为空!!!"<<endl;
        system("pause");
    }
    else
    {
        cout<<"请选择排序方式"<<endl;
        cout<<"1、按职工号进行升序"<<endl;
        cout<<"2、按职工号进行降序"<<endl;
        int select = 0;
        cin>>select;
        for(int i=0;i<m_EmpNum;i++)
        {
            int minOrMax = i;
            for(int j=i+1;j<m_EmpNum;j++)
            {
        if(select == 1)
        {
            if(this->m_EmpArray[minOrMax]->m_Id>this->m_EmpArray[j]->m_Id)
            {
                minOrMax = j;
            }
        }
        else
        {
            if(this->m_EmpArray[minOrMax]->m_Id<this->m_EmpArray[j]->m_Id)
            {
                minOrMax = j;
            }
        }}
            //判断一开始认定 最小值或最大值 是不是计算的最小值或最大值 如果不是则交换数据
        if(i != minOrMax)
        {
            Worker * temp = this->m_EmpArray[i];
            this->m_EmpArray[i] = this->m_EmpArray[minOrMax];
            this->m_EmpArray[minOrMax] = temp;
        }
        }
        this->SaveFile();
        cout<<"排序成功, 排序后的结果为:"<<endl;
        this->ShowEmp();
    }

}
```

### 清空文件

```cpp
void WorkerManager::CleanFile()
{
    cout<<"确定清空"<<endl;
    cout<<"1、yes/2、no"<<endl;
    int select;
    cin>>select;
    if(select == 1)
    {
        ofstream ofs(FILENAME, ios::trunc);
        ofs.close();
        if (this->m_EmpArray != NULL)
        {
            //删除堆区的每个职工对象
            for(int i=0; i<this->m_EmpNum;i++)
            {
                delete this->m_EmpArray[i];
                this->m_EmpArray[i] = NULL;
            }
            delete[] this->m_EmpArray;
            this->m_EmpArray = NULL;
            this->m_EmpNum = 0;
            this->m_FileIsEmpty = true;
        }
        cout<<"清空成功!!!"<<endl;
    }
    system("pause");
}
```

### 主函数

```cpp
#include <iostream>
#include "workermanager.h"
#include "worker.h"
#include "employee.h"
#include "manager.h"
#include "boss.h"
using namespace std;

int main()
{
    WorkerManager wm;
    int option = 0;

    while (true) {
        wm.ShowMenu();
        cout<<"请输入您的选择:"<<endl;
        cin>>option;
        switch (option)
        {
            case 0:
                wm.ExitSystem();
                break;
            case 1:
                wm.AddEmp();
                break;
            case 2:
                wm.ShowEmp();
                break;
            case 3:
                wm.DelEmp();
                break;
            case 4:
                wm.ModEmp();
                break;
            case 5:
                wm.FindEmp();
                break;
            case 6:
                wm.SortEmp();
                break;
            case 7:
                wm.CleanFile();
                break;
            default:
                system("cls");
                break;
        }

    }
    return 0;
}
```
