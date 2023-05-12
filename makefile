FLAGS= -DDEBUG
LIBS= -lm
ALWAYS_REBUILD=makefile

nbody: nbody.o compute.o
	gcc $(FLAGS) $^ -o $@ $(LIBS) -fopenacc
nbody.o: nbody.c planets.h config.h vector.h $(ALWAYS_REBUILD)
	gcc $(FLAGS) -c $< -fopenacc
compute.o: compute.c config.h vector.h $(ALWAYS_REBUILD)
	gcc $(FLAGS) -c $< -fopenacc
clean:
	rm -f *.o nbody 
