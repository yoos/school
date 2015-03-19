#include <GL/glew.h>
#include <GL/glut.h>

#include <glshader.hpp>

float vertices[] = {
  // Axes
  0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f,
  1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f,
  0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f,
  0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f,
  0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f,
  0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f,
};

static void resize(int width, int height)
{
  const float ar = (float) width / (float) height;

  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glFrustum(-ar, ar, -1.0, 1.0, 2.0, 100.0);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity() ;
}

static void display(void)
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glColor3d(1, 0, 0);   // Default color

  // Cube to the right
  glPushMatrix();
  glTranslated(1.3,0.0,-4.);
  glRotated(25,1,1,1);
  glutSolidCube(1.2);
  glPopMatrix();

  // Sphere close to the left
  glColor3d(0, 1, 0);
  glPushMatrix();
  glTranslated(-1.5,0.2,-3.);
  glutSolidSphere(0.8,50,50);
  glPopMatrix();

  // Sphere farther away to the bottom
  glColor3d(0, 1, 1);
  glPushMatrix();
  glTranslated(-0.5,-1.0,-5.);
  glutSolidSphere(0.8,50,50);
  glPopMatrix();

  // Sphere really far away top right
  glColor3d(0, 0, 1);
  glPushMatrix();
  glTranslated(1.0,1.5,-10.);
  glutSolidSphere(0.8,50,50);
  glPopMatrix();

  // Enclose within large sphere so we raytrace everywhere
  glColor3d(1, 1, 1);
  glPushMatrix();
  glutSolidSphere(10,20,20);
  glPopMatrix();

  // Draw vertices
  glColor3d(1, 0, 1);
  glPushMatrix();
  glTranslated(0.0,0.0,-5.);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
  glDrawArrays(GL_LINES, 0, 6);
  glPopMatrix();

  glutSwapBuffers();
}

int main(int argc, char *argv[])
{
  glutInit(&argc, argv);

  // Set up window
  glutInitWindowSize(640,480);
  glutInitWindowPosition(10,10);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
  glutCreateWindow("Smoke");

  // Set up drawing options
  glClearColor(0,0,0,1);

  // Uncomment to enable back face culling
  //glEnable(GL_CULL_FACE);
  //glCullFace(GL_BACK);

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);

  // GLEW
  glewExperimental = GL_TRUE;
  glewInit();

  // Create Vertex Array Object
  GLuint vao;
  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  // Create a Vertex Buffer Object and copy the vertex data to it
  GLuint vbo;
  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

  // Load shaders
  GLuint program = LoadShader("src/smoke.vert", "src/smoke.frag");
  glUseProgram(program);

  // Specify the layout of the vertex data
  GLint posAttrib = glGetAttribLocation(program, "position");
  glEnableVertexAttribArray(posAttrib);
  glVertexAttribPointer(posAttrib, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), 0);

  // Set up draw functions
  glutReshapeFunc(resize);
  glutDisplayFunc(display);

  // Draw
  glutMainLoop();

  // Clean up
  glDeleteProgram(program);
  glDeleteBuffers(1, &vbo);
  glDeleteVertexArrays(1, &vao);

  return EXIT_SUCCESS;
}
