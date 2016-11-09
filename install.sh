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


