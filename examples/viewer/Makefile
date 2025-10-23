
# Makefile for macOS using MacPorts

CXXFILES = viewer.cc og-viewer.cc trackball.cc gl.cc
HEADERS = stb_image.h trackball.h
CXX = clang++
CXXFLAGS += -x c++ -g -O3 -Wall -pedantic -pipe -std=c++20 -Wno-register -Wno-deprecated-declarations -Wno-unused-but-set-variable -I /opt/local/include -I ./glm -I ./spdlog -I ./glad/include
LDFLAGS += -g -O3 -Wall -pedantic -pipe -std=c++20 -L /opt/local/lib -lglfw -lGLEW -framework OpenGL

OBJECTS = $(CXXFILES:.cc=.o)

DEP = $(CXXFILES:.cc=.d)

all: viewer ogviewer

viewer: viewer.o trackball.o gl.o
	$(CXX) $(LDFLAGS) -o viewer trackball.o viewer.o gl.o

ogviewer: og-viewer.o trackball.o
	$(CXX) $(LDFLAGS) -o ogviewer trackball.o og-viewer.o

%.o: %.cc
	$(CXX) $(CXXFLAGS) -c $< -o $@ -MMD -MP

clean:
	-rm $(OBJECTS)
	-rm $(DEP)

spotless: clean
	-rm viewer ogviewer

-include $(DEP)
