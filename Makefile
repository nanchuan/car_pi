#source file
# Դ�ļ����Զ������� .c �� .cpp �ļ�������Ŀ�궨��Ϊͬ�� .o �ļ�
SOURCE  := $(wildcard *.c) $(wildcard *.cpp)
OBJS    := $(patsubst %.c,%.o,$(patsubst %.cpp,%.o,$(SOURCE)))
 
#target you can change test to what you want
# Ŀ���ļ�����������������Ҫ��ִ���ļ���
TARGET  := app
 
#compile and lib parameter
# �������
CC      := gcc
LIBS    := -lwiringPi -lpigpio -lrt -lm 
LDFLAGS := -pthread
DEFINES :=
INCLUDE := -I.
CFLAGS  := -g -Wall -O3 $(DEFINES) $(INCLUDE) 
CXXFLAGS:= $(CFLAGS) -DHAVE_CONFIG_H
 
 
#i think you should do anything here
# ����Ļ����ϲ���Ҫ���κθĶ���
.PHONY : everything objs clean veryclean rebuild
 
everything : $(TARGET)
 
all : $(TARGET)
 
objs : $(OBJS)
 
rebuild: veryclean everything
               
clean :
	rm -fr *.so
	rm -fr *.o
   
veryclean : clean
	rm -fr $(TARGET)
 
$(TARGET) : $(OBJS) 
	$(CC) $(CXXFLAGS) -o $@ $(OBJS) $(LDFLAGS) $(LIBS)
 