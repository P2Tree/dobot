<<<<<<< HEAD
#!/bin/bash
updir=/home/pwe/catkin_ws/src/dobot    #要上传的文件夹
todir=catkin_ws/src/dobot          #目标文件夹，必须用相对路径
ip=192.168.1.43      #服务器
user=ubuntu          #ftp用户名
password=1406        #ftp密码
sss=`find $updir -type d -printf $todir/'%P\n'| awk '{if ($0 == "")next;print "mkdir " $0}'`
aaa=`find $updir -type f -printf 'put %p %P \n'`
ftp -nv $ip <<EOF
user $user $password
type binary
prompt
$sss
cd $todir
$aaa
rm install.sh
quit
EOF
=======
#!/bin/sh

trans_date='date +%y%m%d%H%M'
target='runDobot'
local_addr='./bin'
remote_addr='~/driver'
remote_ipaddr='192.168.1.43'

ftp_user='ubuntu'
ftp_password='1406'

echo "
open ${remote_ipaddr}
prompt
user ${ftp_user} ${ftp_password}
lcd ${local_addr}
cd ${remote_addr}
passive on
binary
mput ${target}
close
bye
"|ftp -v -n |sed 's/^/>/g' >> run.log

if [ -s run.log ]
then
    echo "SYSTEM: ftp login success."
    SEARCH=`grep 'bytes sent in' run.log`
    if [ $? -eq 0 ]
    then
        echo "FTP: \033[1;33;5msuccess\033[0m to transfer ${local_addr}/${target} to ${remote_ipaddr}${remote_addr}"
        rm run.log
    else
        echo "FTP: \033[1;31;5mfail\033[0m to transfer ${local_addr}/${target} to ${remote_ipaddr}${remote_addr}"
        cat run.log
        rm run.log
    fi
else
    echo "SYSTEM: ftp login \033[1;31;5mFAIL\033[0m."
    exit 1
fi

>>>>>>> edb5b1272eb5b4515f58b22f6aec51c87c7ff414

