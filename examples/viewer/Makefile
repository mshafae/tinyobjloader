
# Makefile for macOS using MacPorts

CFLAGS += -std=c++17 -Wno-register -Wno-deprecated-declarations -I /opt/local/include
LDFLAGS += -L /opt/local/lib -lglfw -lGLEW -framework OpenGL
default viewer: viewer.o trackball.o
	clang++ $(CFLAGS) trackball.o viewer.o $(LDFLAGS) -o viewer

ogviewer: og-viewer.cc trackball.o
	clang++ $(CFLAGS) trackball.o og-viewer.o $(LDFLAGS) -o ogviewer

trackball.o: trackball.cc
	clang++ $(CFLAGS) -c $<

viewer.o: viewer.cc
	clang++ $(CFLAGS) -c $<

og-viewer.o: og-viewer.cc
	clang++ $(CFLAGS) -c $<


clean:
	-rm trackball.o viewer.o og-viewer.o
	-rm viewer ogviewer