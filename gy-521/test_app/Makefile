.PHONY: clean

all: App

CC=gcc

SOURCE=mpuApp.c
OBJ=mpuApp.o

App: $(OBJ)
	$(CC) -o $@ $^

%.o: %.c
	$(CC) -c $< -o $@
clean:
	rm -rf App
	rm -rf *.o
