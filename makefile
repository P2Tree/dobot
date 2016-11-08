
DIR_INC = ./
DIR_SRC = ./
DIR_OBJ = ./
DIR_BIN = ./

TAG = runDobot

CC = g++
CFLAGS = -Wall

dobotDriver.o : dobotDriver.cpp dobotDriver.hpp
	${CC} -c ${CFLAGS} dobotDriver.cpp

runDobot.o : runDobot.cpp dobotDriver.cpp dobotDriver.hpp
	$(CC) -c ${CFLAGS} runDobot.cpp

${TAG} : dobotDriver.o runDobot.o
	$(CC) -o $(CFLAGS)

.PHONY : clean cleanall install

clean :
	-rm -rf *.o

cleanall:
	-rm -rf *.o ${TAG}

install :
	./

	
