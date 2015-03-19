// Code lifted from http://www.nexcius.net/2012/11/20/how-to-load-a-glsl-shader-in-opengl-using-c/

#ifndef GLSHADER_H
#define GLSHADER_H

#include "GL/glew.h"

GLuint LoadShader(const char *vertex_path, const char *fragment_path);

#endif
