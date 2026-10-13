import math
import threading
import time
import sys
import os
import numpy as np

try:
    from OpenGL.GL import *
    from OpenGL.GLU import *
    from OpenGL.GLUT import *
except ImportError:
    raise ImportError(
        "PyOpenGL and FreeGLUT are required. Install via: pip install PyOpenGL PyOpenGL_accelerate"
    )

NUMCONTROLS = 4
MAX_TEXTURE = 40

# Global Mutexes (replacing boost::mutex)
state_mutex = threading.Lock()
time_mutex = threading.Lock()
control_mutex = threading.Lock()
ok_mutex = threading.Lock()
gl_mutex = threading.Lock()


class MATLABMatrix:
    """Lightweight Python equivalent of the custom MATLAB C++ matrix wrapper."""

    def __init__(self, rows=1, cols=1, name=""):
        self.name = name
        self.data = np.zeros((rows, cols))

    def zeros(self, rows, cols, name=""):
        self.name = name
        self.data = np.zeros((rows, cols))

    def set(self, r, c, val):
        # Converting 1-based MATLAB indexing to 0-based Python indexing
        self.data[r - 1, c - 1] = val

    def get(self, r, c):
        return self.data[r - 1, c - 1]

    def overwrite(self, other):
        self.data = np.copy(other.data)

    def disp(self):
        print(f"Matrix {self.name}:\n{self.data}")


class StateHistory:

    def __init__(self):
        self.advance = 0
        self.objects = 0
        self.filesize = 0
        self.rowdx = 0
        self.ok = 0
        self.T = 0.0
        self.control = np.zeros(NUMCONTROLS)
        self.positionscale = MATLABMatrix()
        self.cg = MATLABMatrix()
        self.ptp = MATLABMatrix()
        self._numobjects = 0

    def Initialize(self, number_of_objects):
        self.ok = 0
        self.setTime(0.0)
        self._numobjects = number_of_objects
        self.objects = number_of_objects

        self.cg.zeros(3, self._numobjects, "state.cg")
        self.ptp.zeros(3, self._numobjects, "state.ptp")

        with control_mutex:
            self.control.fill(0.0)

        self.positionscale.zeros(self._numobjects, 1, "positionscale")
        self.advance = 0
        self.ok = 1

    def setTime(self, simtime):
        with time_mutex:
            self.T = simtime

    def UpdateRender(self, simtime, cgin, ptpin, object_number, keyboard_out):
        self.setTime(simtime)

        with state_mutex:
            scale = self.positionscale.get(object_number, 1)
            for idx in range(1, 4):
                self.cg.set(
                    idx, object_number, scale * cgin.get(idx, 1)
                )
                self.ptp.set(idx, object_number, ptpin.get(idx, 1))

        with control_mutex:
            for i in range(NUMCONTROLS):
                keyboard_out[i] = self.control[i]

    def getState(self, cgout, ptpout):
        with state_mutex:
            cgout.overwrite(self.cg)
            ptpout.overwrite(self.ptp)


class CameraControl:

    def __init__(self):
        self.objects = 0
        self.ctype = 0
        self.camera_zoffset = 0.0
        self.rearview = 50.0
        self.cg = np.zeros(3)
        self.ptp = np.zeros(3)
        self.freepos = np.zeros(3)
        self.freetarget = np.array([-10.0, 0.0, -1.0])
        self.targetdistance = 0.0
        self.pos = np.zeros(3)
        self.target = np.zeros(3)
        self.up = np.array([0.0, 0.0, 1.0])
        self.xcamera = np.zeros(3)
        self.psi = 0.0
        self.theta = 0.0

    def Initialize(
        self,
        num_objects,
        state_cg,
        state_ptp,
        default_camera,
        camera_offset_zaxis,
    ):
        self.camera_zoffset = camera_offset_zaxis
        self.objects = num_objects
        self.ctype = default_camera
        self.rearview = 50.0

        self.cg[0] = state_cg.get(1, 1)
        self.cg[1] = state_cg.get(2, 1)
        self.cg[2] = state_cg.get(3, 1)
        self.ptp[0] = state_ptp.get(1, 1)
        self.ptp[1] = state_ptp.get(2, 1)
        self.ptp[2] = state_ptp.get(3, 1)
        self.psi = self.ptp[2]

        self.freepos.fill(0.0)
        self.freetarget = np.array([-10.0, 0.0, -1.0])

        self.pos[0] = self.cg[0] - self.rearview * math.cos(self.psi)
        self.pos[1] = self.cg[1] - self.rearview * math.sin(self.psi)
        self.pos[2] = self.cg[2]
        self.target = np.copy(self.cg)

        self.ComputeRange()
        self.theta = math.asin(-self.xcamera[2])
        self.psi = math.atan2(self.xcamera[1], self.xcamera[0])
        self.ComputeUp()
        self.ctype -= 1
        self.SwitchCamera()

    def SwitchCamera(self):
        if self.ctype == 2 * self.objects:
            self.freepos = np.copy(self.pos)
            self.freetarget = np.copy(self.target)

        self.ctype += 1
        if self.ctype > 2 * self.objects:
            self.ctype = 0

        if self.ctype == 2 * self.objects:
            self.pos = np.copy(self.freepos)
            self.target = np.copy(self.freetarget)
            self.ComputeRange()
            self.theta = math.asin(-self.xcamera[2])
            self.psi = math.atan2(self.xcamera[1], self.xcamera[0])
            self.ComputeUp()
            print("Free Camera")
        elif self.ctype < self.objects:
            print(f"Following Object = {self.ctype + 1}")
        elif self.ctype < 2 * self.objects:
            print(f"Origin Camera Pointing at Object = {self.ctype - self.objects + 1}")

    def ComputeUp(self):
        self.up[0] = -math.cos(self.psi) * math.sin(self.theta)
        self.up[1] = -math.sin(self.psi) * math.sin(self.theta)
        self.up[2] = -math.cos(self.theta)

    def ComputeRange(self):
        self.xcamera = self.target - self.pos
        self.targetdistance = np.linalg.norm(self.xcamera)
        if self.targetdistance > 0:
            self.xcamera /= self.targetdistance

    def zoom(self, direction):
        if self.ctype < self.objects:
            self.rearview -= direction
            if self.rearview < 1:
                self.rearview = 1
        else:
            self.pos += direction * self.xcamera
            self.target += direction * self.xcamera
            self.ComputeRange()

    def pitch(self, direction):
        self.theta -= direction * 0.1
        self.ComputeForward()
        self.ComputeUp()
        if self.ctype >= self.objects:
            self.ComputeNewTarget()

    def yaw(self, direction):
        self.psi -= direction * 0.1
        self.ComputeForward()
        self.ComputeUp()
        if self.ctype >= self.objects:
            self.ComputeNewTarget()

    def ComputeForward(self):
        self.xcamera[0] = math.cos(self.psi) * math.cos(self.theta)
        self.xcamera[1] = math.cos(self.theta) * math.sin(self.psi)
        self.xcamera[2] = -math.sin(self.theta)

    def ComputeNewTarget(self):
        self.target = self.pos + self.targetdistance * self.xcamera

    def Update(self, state_history):
        if self.ctype < self.objects:
            idx = self.ctype
            for ii in range(3):
                self.cg[ii] = state_history.cg.get(ii + 1, idx + 1)
                self.ptp[ii] = state_history.ptp.get(ii + 1, idx + 1)
                self.target[ii] = self.cg[ii]

            ctheta = math.cos(self.theta)
            stheta = math.sin(self.theta)
            cpsi = math.cos(self.psi)
            spsi = math.sin(self.psi)

            self.pos[0] = self.cg[0] - ctheta * cpsi * self.rearview
            self.pos[1] = self.cg[1] - ctheta * spsi * self.rearview
            self.pos[2] = self.cg[2] + stheta * self.rearview

            self.ComputeRange()
            self.theta = math.asin(-self.xcamera[2])
            self.psi = math.atan2(self.xcamera[1], self.xcamera[0])
            self.ComputeUp()
        elif self.ctype < 2 * self.objects:
            idx = self.ctype - self.objects
            for ii in range(3):
                self.target[ii] = state_history.cg.get(ii + 1, idx + 1)
                self.pos[ii] = 0.0
            self.ComputeRange()
            self.theta = math.asin(-self.xcamera[2])
            self.psi = math.atan2(self.xcamera[1], self.xcamera[0])
            self.ComputeUp()
            self.pos[2] -= self.camera_zoffset


class MouseControl:

    def __init__(self):
        self.mousex = 0
        self.mousey = 0
        self.prevTime = 0.0
        self.GetNewMouse = 1.0 / 120.0


class KeyboardControl:

    def __init__(self):
        self.lightflag = 0


class RenderControl:

    def __init__(self):
        self.ok = 0
        self.counter = 0
        self.figure = 0
        self.Width = 800
        self.Height = 600
        self.Farplane = 1000
        self.savepics = 0
        self.ready = 0
        self.simtime = 0.0
        self.initial = 0.0
        self.mouse = MouseControl()
        self.camera = CameraControl()
        self.state = StateHistory()
        self.keyboard = KeyboardControl()

    def loop_init(self, fileroot=None, inFarplane=1000, inWidth=800, inHeight=600):
        print("Initializing RenderControl Rendering Environment")
        objectfile = (
            f"{fileroot}Input_Files/Render.txt"
            if fileroot
            else "Input_Files/Render.txt"
        )

        try:
            lines = []
            with open(objectfile, "r") as f:
                for line in f:
                    # Strip comments and whitespace after '!'
                    part = line.split("!")[0].strip()
                    if part:
                        lines.append(part)
        except FileNotFoundError:
            print(f"Error Opening File = {objectfile}")
            return

        self.savepics = int(lines[0])
        num_objects = int(lines[1])
        print(f"Number of Objects to Render = {num_objects}")

        self.state.Initialize(num_objects)
        camera_offset_zaxis = float(lines[2])
        default_camera = int(lines[3])
        self.camera.Initialize(
            num_objects,
            self.state.cg,
            self.state.ptp,
            default_camera,
            camera_offset_zaxis,
        )

        self.Farplane = inFarplane
        self.Width = inWidth
        self.Height = inHeight

        # Parse object properties (each object entry uses 3 lines: file, scale, position scale)
        idx = 4
        for ii in range(num_objects):
            obj_path = lines[idx]
            scale_factor = float(lines[idx + 1])
            pos_scale = float(lines[idx + 2])
            print(
                f"Loading Object {ii+1}: {obj_path} (Scale: {scale_factor}, Pos Scale: {pos_scale})"
            )

            self.state.positionscale.set(ii + 1, 1, pos_scale)
            idx += 3

        # Window Initialization
        glutInit()
        glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH)
        glutInitWindowSize(self.Width, self.Height)
        glutInitWindowPosition(100, 100)
        self.figure = glutCreateWindow(b"Multi Object Visualizer (Python)")

        glEnable(GL_TEXTURE_2D)
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glClearDepth(1.0)
        glDepthFunc(GL_LESS)
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(
            60.0, float(self.Width) / float(self.Height), 0.1, self.Farplane
        )
        glMatrixMode(GL_MODELVIEW)

        glutDisplayFunc(self.draw_gl_scene)
        glutIdleFunc(self.main_loop)
        glutReshapeFunc(self.resize_gl_scene)
        glutKeyboardFunc(self.key_pressed)
        glutMotionFunc(self.mouse_motion)
        glutPassiveMotionFunc(self.save_mouse)
        glutMouseFunc(self.mouse_click)

        self.ready = 1
        print("Visualizer Initialized and Running...")
        glutMainLoop()

    def resize_gl_scene(self, width, height):
        if height == 0:
            height = 1
        self.Width = width
        self.Height = height
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(60.0, float(width) / float(height), 0.1, self.Farplane)
        glMatrixMode(GL_MODELVIEW)

    def main_loop(self):
        glutPostRedisplay()

    def key_pressed(self, key, x, y):
        k = key.decode("utf-8") if isinstance(key, bytes) else key
        control_keys = ['u', 'j', 'h', 'k', 't', 'g', 'm', 'n']

        if k in control_keys:
            with control_mutex:
                if k == 'u':
                    self.state.control[2] -= 0.1
                elif k == 'j':
                    self.state.control[2] += 0.1
                elif k == 'h':
                    self.state.control[1] -= 0.1
                elif k == 'k':
                    self.state.control[1] += 0.1
                elif k == 't':
                    self.state.control[0] += 0.1
                elif k == 'g':
                    self.state.control[0] -= 0.1
                elif k == 'm':
                    self.state.control[3] += 0.1
                elif k == 'n':
                    self.state.control[3] -= 0.1

                for i in range(NUMCONTROLS):
                    if abs(self.state.control[i]) > 1.0:
                        self.state.control[i] = math.copysign(1.0, self.state.control[i])
        if k == '\x1b' or k == 'q':
            print("Quitting RenderControl")
            self.ok = 0
            self.ready = 0
            try:
                glutDestroyWindow(self.figure)
            except Exception:
                pass
            os._exit(0)  # Cleanly terminates process without CTypes exception traceback
        elif k == 'c':
            self.camera.SwitchCamera()
        elif k == 'o':
            self.camera.zoom(-1)
        elif k == 'i':
            self.camera.zoom(1)
        elif k == 'w':
            self.camera.pitch(1)
        elif k == 's':
            self.camera.pitch(-1)
        elif k == 'a':
            self.camera.yaw(-1)
        elif k == 'd':
            self.camera.yaw(1)
        elif k == ' ':
            print("Playing...")
            self.state.advance = 1
        elif k == 'l':
            if self.keyboard.lightflag == 0:
                glEnable(GL_LIGHTING)
                self.keyboard.lightflag = 1
            else:
                glDisable(GL_LIGHTING)
                self.keyboard.lightflag = 0

    def save_mouse(self, x, y):
        self.mouse.mousex = x
        self.mouse.mousey = y

    def mouse_click(self, button, state, x, y):
        if button == 3:
            self.camera.zoom(0.5)
        elif button == 4:
            self.camera.zoom(-0.5)

    def mouse_motion(self, x, y):
        delx = (x - self.mouse.mousex) / 100.0
        dely = (y - self.mouse.mousey) / 100.0
        self.camera.yaw(delx)
        self.camera.pitch(-dely)
        self.mouse.mousex = x
        self.mouse.mousey = y

    def draw_gl_scene(self):
        if self.initial == 0.0:
            self.initial = glutGet(GLUT_ELAPSED_TIME)

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        with state_mutex:
            self.camera.Update(self.state)

        gluLookAt(
            self.camera.pos[0], self.camera.pos[1], self.camera.pos[2],
            self.camera.target[0], self.camera.target[1], self.camera.target[2],
            self.camera.up[0], self.camera.up[1], self.camera.up[2]
        )

        # Draw Objects
        for ii in range(1, self.state.objects + 1):
            with state_mutex:
                cg = [self.state.cg.get(j, ii) for j in range(1, 4)]
                ptp = [self.state.ptp.get(j, ii) for j in range(1, 4)]

            glTranslatef(cg[0], cg[1], cg[2])
            glRotatef(ptp[2] * (180.0 / math.pi), 0, 0, 1)
            glRotatef(ptp[1] * (180.0 / math.pi), 0, 1, 0)
            glRotatef(ptp[0] * (180.0 / math.pi), 1, 0, 0)

            # Placeholder drawing for OBJ geometry (replace with your OBJ vertex list rendering)
            glutWireCube(2.0)

            glRotatef(-ptp[0] * (180.0 / math.pi), 1, 0, 0)
            glRotatef(-ptp[1] * (180.0 / math.pi), 0, 1, 0)
            glRotatef(-ptp[2] * (180.0 / math.pi), 0, 0, 1)
            glTranslatef(-cg[0], -cg[1], -cg[2])

        glutSwapBuffers()


# Global Handle Instance
glhandle_g = RenderControl()