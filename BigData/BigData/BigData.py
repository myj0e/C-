#-*- encoding:utf-8 -*-
import re
def function(Type):
    file=open('input.c','r+')
    Function=' *'+Type+' +(\w*)\s*\( *(.*\))'
    text=''
    I=re.compile(Function)
    for line in file:
        text=text+line
    func=list(set(re.findall(I,text)))
    file.close()
    return list(func)


#All types of C
types=['int','char','void','double','short','float','u8','u16']
func=[]
#Gether all types' returns into func
for Type in types:
    func=func+function(Type)
print("func:")
print(func)

#Select all functions into a list
fun=[]
par=[]
para=[]
param=[]
sum=[]
parame=" +([\S]*) *[,\)]"
p=re.compile(parame)
for i in func:
    par=[]
    fun.append(i[0])
    param.append(i[1])
    par=re.findall(p,i[1])
    para=para+par
    sum.append((i[0],par))
print(sum)
print("所有函数：")
print(fun)
while '' in para:
    para.remove('')
print("所有参数：")
para=list(set(para))
print(para)
text=''
for i in para:
    text=text+i+','
text=text[:-1]
file=open("output.txt","a+")
file.write(','+text+'\n')
for i in sum:
    count=[]
    text=[]
    for j in range(len(para)):
        count.append("")
    text2=','* len(para)
    text=str(i[0])
    for j in i[1]:
        for k in range(len(para)):
            if j==para[k]:
                count[k]="*"
    for j in range(len(para)):
        text=text+text2[j]+count[j]
    text=text+'\n'
    file.write(text)

file.close()
