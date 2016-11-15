
DIR_INC = ./
DIR_SRC = ./
DIR_OBJ = ./obj
DIR_BIN = ./bin

SRC = ${wildcard ${DIR_SRC}/*.c}
OBJ = $(patsubst %.c, ${DIR_OBJ}/%.o, $(notdir ${SRC}))

TAG = runDobot
BIN_TAG = ${DIR_BIN}/${TAG}

CC = g++
CFLAGS = -Wall

${BIN_TAG} : dobotDriver.o runDobot.o
	$(CC) -o ${BIN_TAG} runDobot.o dobotDriver.o

dobotDriver.o : dobotDriver.cpp dobotDriver.hpp
	${CC} -c ${CFLAGS} dobotDriver.cpp

runDobot.o : runDobot.cpp dobotDriver.cpp dobotDriver.hpp
	$(CC) -c ${CFLAGS} runDobot.cpp


.PHONY : clean cleanall install

clean :
	-rm -f *.o

cleanall:
	-rm -f *.o ${TAG} ${BIN_TAG}

install :
	./install.sh

	
