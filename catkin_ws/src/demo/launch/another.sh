#!/bin/bash
# 赋值等号两边不能有空格
var="test_var"

function test_fuc() {
  var=$#
  echo $*
}
a=1

#echo "$var"
#test_fuc another cykablyat
#echo "$var"
#while (($[a]<=10))
#do
#    echo $a
##    a=`expr $a + 1`
#done
#read DF
#echo $DF $DF
if [ $(ps -ef | grep -c "ssh") -gt 1 ]; then echo "true"; fi