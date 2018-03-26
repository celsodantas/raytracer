
BUILDDIR = build
CPP_COMPILER = llvm-g++
CPP_FLAGS	= -Wall -Wpedantic -std=c++1z -framework SDL2
DEBUGFLAG= -g
HEADERS= -Iincludes
CPPFILES= main.cpp

all: compile

compile:
	$(CPP_COMPILER) $(CPP_FLAGS) $(HEADERS) $(CPPFILES) $(DEBUGFLAG) -o $(BUILDDIR)/raytracer
	chmod +x $(BUILDDIR)/raytracer

clean:
	rm -fr $(BUILDDIR)/*.o*

build_and_run: clean compile
	./$(BUILDDIR)/raytracer
