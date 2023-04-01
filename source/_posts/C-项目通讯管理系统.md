---
title: C++项目通讯管理系统
typora-root-url: ..\imgs
date: 2022-08-30 20:51:14
tags: C/C++
categories: 
        - 程序设计
        - C/C++
---

## 显示菜单

```C++
void showMenu()
{
    cout<<"*********************"<<endl;
    cout<<"**** 1、添加联系人 ****"<<endl;
    cout<<"**** 2、显示联系人 ****"<<endl;
    cout<<"**** 3、删除联系人 ****"<<endl;
    cout<<"**** 4、查找联系人 ****"<<endl;
    cout<<"**** 5、修改联系人 ****"<<endl;
    cout<<"**** 6、清空联系人 ****"<<endl;
    cout<<"**** 0、退出通讯录 ****"<<endl;
    cout<<"*********************"<<endl;
}
```

## 添加联系人

### 设计联系人结构体

```C++
struct Person
{
    string name;
    int sex;
    int age;
    int phone;
    string add;
};
```

### 设计通讯录结构体

```c++
#define MAX 1000

struct Addressbooks
{
    Person array[1000];
    int size;
};
```

## 显示联系人

```C++
void showperson(Addressbooks *abs)
{
    if(abs->size == 0)
    {
        cout<<"联系人为空!!!"<<endl;
    }
    else
    {
        cout<<"联系人数量为"<<abs->size+1<<endl;
        for(int i=0;i<abs->size;i++)
        {
            cout<<"------第"<<i+1<<"个联系人------"<<endl;
            cout<<"姓名:"<<abs->array[i].name<<endl;
            cout<<"性别:"<<abs->array[i].sex<<endl;
            cout<<"年龄:"<<abs->array[i].age<<endl;
            cout<<"电话:"<<abs->array[i].phone<<endl;
            cout<<"地址:"<<abs->array[i].add<<endl;
        }
    }
}
```

## 删除联系人

```C++
int isExist(Addressbooks *abs, string name)
{
    for(int i=0;i<abs->size;i++) {
        if (abs->array[i].name == name)
        {
            return i;
        }
    }
    return -1;
}
void DelPerson(Addressbooks *abs)
{
    string name;
    cout<<"请输入删除人的姓名:"<<endl;
    cin>>name;
    int re = isExist(abs, name);
    if (re != -1)
    {
        for(int i=re;i<abs->size;i++)
        {
            abs->array[i] = abs->array[i+1];
        }
        abs->size--;
        cout<<"删除成功!!!"<<endl;
    }
    else
    {
        cout<<"没有此人!!!"<<endl;
    }
}
```

## 查找联系人

```C++
void SelectPerson(Addressbooks *abs)
{
    string name;
    cout<<"请输入要查找的人的姓名:"<<endl;
    cin>>name;
    int res = isExist(abs, name);
    if (res != -1)
    {
        cout<<"查到此人!!!"<<endl;
        cout<<"姓名:"<<abs->array[res].name<<endl;
        cout<<"性别:"<<abs->array[res].sex<<endl;
        cout<<"年龄:"<<abs->array[res].age<<endl;
        cout<<"电话:"<<abs->array[res].phone<<endl;
        cout<<"地址:"<<abs->array[res].add<<endl;
    }
    else
    {
        cout<<"查无此人!!!"<<endl;
    }
}
```

## 修改联系人

```C++
void ModifyPerson(Addressbooks *abs)
{
    string name;
    cout<<"请输入要修改的人的姓名:"<<endl;
    cin>>name;
    int res = isExist(abs, name);
    if (res != -1)
    {
        string new_name;
        cout<<"修改后姓名"<<endl;
        cin>>new_name;
        abs->array[res].name = new_name;
        cout << "请输入性别:" << endl;
        cout << "男/女" << endl;
        string sex = "男";
        while (true)
        {
            cin >> sex;
            //如果输入正确值则退出，输入有误可重新输入
            if (sex == "男" || sex == "女")
            {
                abs->array[res].sex = sex;
                break;
            }
            cout<<"输入错误,请重新配置"<<endl;
        }
        int new_age;
        cout<<"修改后年龄"<<endl;
        cin>>new_age;
        abs->array[res].age = new_age;
        //电话
        cout<<"修改后电话:"<<endl;
        string new_phone;
        cin>>new_phone;
        abs->array[res].phone = new_phone;
        //地址
        cout<<"修改后地址:"<<endl;
        string new_add;
        cin>>new_add;
        abs->array[res].add = new_add;

        cout<<"修改成功"<<endl;

    }
    else
    {
        cout<<"查无此人!!!"<<endl;
    }
}
```

## 清空联系人

```C++
void ClcPerson(Addressbooks *abs)
{
    abs->size = 0;
    cout<<"联系人已清空!!!"<<endl;
}
```

## 主函数

```C++
int main()
{
    //创建通讯录结构体变量
    Addressbooks abs;
    //初始化通讯录人员个数
    abs.size = 0;
    int select = 0;

    while (true)
    {
        showMenu();
        cin >> select;
        switch (select)
        {
        case 1:
            AddPerson(&abs); //利用地址传传递，可以修改形参
            break;
        case 2:
            ShowPerson(&abs);
            break;
        case 3:
            DelPerson(&abs);
            break;
        case 4:
            SelectPerson(&abs);
            break;
        case 5:
            ModifyPerson(&abs);
            break;
        case 6:
            ClcPerson(&abs);
            break;
        case 0:
            cout << "**** 欢迎下次使用!!! ****" << endl;
            system("pause");
            return 0;
            break;
        default:
            break;
        }
    }
}
```

