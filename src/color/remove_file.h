#include <unistd.h>
#include <stdio.h>
#include <dirent.h>
#include <string.h>
#include <sys/stat.h>
#include <stdlib.h>

void printdir(const char *dir, int depth)
{
    DIR *dp;    //类似于FILE*
    struct dirent *entry;   //entry->d_type：文件类型
    //记录和文件相关的一些信息
    struct stat statbuf;

    //opendir()：打开一个目录并建立一个目录流，
    //成功则返回一个指向DIR结构的指针
    if((dp = opendir(dir)) == NULL)
    {
        fprintf(stderr, "cannot open directory:%s\n", dir);
        return;
    }

    //将工作目录更改到dir
    chdir(dir);

    //readdir()返回一个指针，该指针指向的结构里保存着dp中下一个目录项的有关资料
    while((entry = readdir(dp)) != NULL)
    {
        //获取一些与文件相关的信息
        lstat(entry->d_name, &statbuf);
        //若为目录
        if(S_ISDIR(statbuf.st_mode))
        {
            //排除当前目录和上级目录，避免死循环
            if(strcmp(".", entry->d_name) == 0 || strcmp("..", entry->d_name) == 0)
            {
                continue;
            }
            printf("%*s%s/\n", depth, "", entry->d_name);
            printdir(entry->d_name, depth + 4);
        }
        else
        {
            //printf("%*s%s\n", depth, "", entry->d_name);

            // 删除文件
            if( remove(entry->d_name) != 0 )
                perror("remove");
        }
    }
    //系统调用，功能就像cd一样
    chdir("..");
    //关闭目录流，释放相关的资源
    closedir(dp);
}
