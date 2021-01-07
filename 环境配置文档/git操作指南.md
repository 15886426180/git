# git基本运行指南
----
## 1.创建自己的版本库
* 在终端输入  
* mkdir github  //新建一个文件夹
* cd github //进入新建文件夹
* pwd   //查看自己的目录
* git init //初始化 
## 2.上传文件
* 在终端输入
* nano readme.txt //创建一个readme文件 在里面写入你想要保存的东西  
* git add readme.txt //将文件添加到Git仓库
* git commit -m "这里填写上传代码的备注" //给上传代码添加备注
## 3.返回旧版本
* 在终端输入
* nano readme.txt //修改文件
* //readme文件上传
* git log //输入后可以看到历史版本
* //HEAD表示当前版本 上一个版本HEAD^
上100个版本为HEAD~100
* git reset --hard HEAD^ //返回上一个版本
* git reset --hard +版本号 //可以返回到你想要的版本
## 4.删除文件
* 在终端输入
* rm readme.txt //删除本地文件
* git rm readme.txt //删除git上文件
* git commit
## 5.添加远程库
* 在github新建一个仓库//我这里以name命名
* git remote add origin git@github.com:这里填入你的github账户名/name.git
* git push -u origin master//上传之后就可以在github看到你的上传的文件
* git push origin master//之后的提交都使用这一行即可
## 6.从远程库克隆
* git clone git@github.com:你的github账户名/name.git
## 7.分支管理
* git checkout -b dev//创建一个dev分支
* git branch // 查看分支 当前分支则前面标*  
* //上传你想要在dev上传的文件
* git checkout master //切回到master
* git merge dev //合并分支到master
* git branch -d dev//删除dev分支
* git switch -c dev//创建并切换新分支
* git switch master//切换到master分支