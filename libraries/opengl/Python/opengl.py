import math
import os
import threading
import time
import numpy as np

# Force PyOpenGL to use the X11 platform backend (fixes Linux/Wayland context errors)
# Will need to change this if you are using a different platform (e.g., Windows or macOS)
os.environ["PYOPENGL_PLATFORM"] = "x11"

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

# Global Mutexes
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
                self.cg.set(idx, object_number, scale * cgin.get(idx, 1))
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


class OBJSETUP:

    def __init__(self):
        self.objects = 0
        self.ok = 0
        self.texture = []
        self.gl_list = []

    def Initialize(self, NumObjects):
        print("Initializing OBJ Import")
        self.texture = list(glGenTextures(MAX_TEXTURE))
        self.gl_list = [glGenLists(1) for _ in range(MAX_TEXTURE)]
        print("OBJ Import Initialized")
        self.ok = 1

    def Load(self, filename, number, scale_flag, amount):
        self.ok = 1
        print(f"Importing OBJ File: {filename}")

        # Extract directory path relative to file location
        directory = os.path.dirname(filename)
        directory = (directory + "/") if directory else ""

        try:
            with open(filename, "r") as f:
                content = f.read()
        except FileNotFoundError:
            print(f"File not found: {filename}")
            self.ok = 0
            return

        vertices = []
        normals = []
        texcoords = []
        triangles = []
        material_file = None
        texture_file = None

        for line in content.splitlines():
            parts = line.strip().split()
            if not parts:
                continue
            if parts[0] == "v":
                vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
            elif parts[0] == "vn":
                normals.append([float(parts[1]), float(parts[2]), float(parts[3])])
            elif parts[0] == "vt":
                texcoords.append([float(parts[1]), float(parts[2])])
            elif parts[0] == "mtllib":
                material_file = parts[1]
            elif parts[0] == "f":
                face_verts = []
                for p in parts[1:4]:
                    vals = p.split("/")
                    v_idx = int(vals[0]) if vals[0] else 0
                    vt_idx = int(vals[1]) if len(vals) > 1 and vals[1] else 0
                    vn_idx = int(vals[2]) if len(vals) > 2 and vals[2] else 0
                    face_verts.append((v_idx, vt_idx, vn_idx))
                triangles.append(face_verts)

        # Parse MTL file if declared
        if material_file:
            mtl_path = directory + material_file
            try:
                with open(mtl_path, "r") as mf:
                    for mline in mf:
                        mparts = mline.strip().split()
                        if len(mparts) > 1 and mparts[0] == "map_Kd":
                            texture_file = mparts[1]
                            break
            except FileNotFoundError:
                print(f"Material file not found: {mtl_path}")

        # Load BMP texture if referenced in MTL
        if texture_file:
            tex_path = directory + texture_file
            self.LoadTexture(tex_path, self.texture[number - 1])

        amount = amount if amount != 0 else 1.0

        # Compile OpenGL Display List
        glNewList(self.gl_list[number - 1], GL_COMPILE)
        glEnable(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D, self.texture[number - 1])
        glFrontFace(GL_CCW)

        glBegin(GL_TRIANGLES)
        for tri in triangles:
            for v_idx, vt_idx, vn_idx in tri:
                if vn_idx > 0 and vn_idx <= len(normals):
                    n = normals[vn_idx - 1]
                    glNormal3f(n[0] * amount, n[1] * amount, n[2] * amount)
                if vt_idx > 0 and vt_idx <= len(texcoords):
                    t = texcoords[vt_idx - 1]
                    glTexCoord2f(t[0], t[1])
                if v_idx > 0 and v_idx <= len(vertices):
                    v = vertices[v_idx - 1]
                    glVertex3f(v[0] * amount, v[1] * amount, v[2] * amount)
        glEnd()

        glDisable(GL_TEXTURE_2D)
        glEndList()
        print(f"Successfully compiled object {number} ({filename})")

    def LoadTexture(self, filename, tex_id):
        try:
            with open(filename, "rb") as f:
                header = f.read(54)
                if len(header) < 54 or header[:2] != b"BM":
                    print(f"Not a valid BMP file: {filename}")
                    return

                width = int.from_bytes(header[18:22], byteorder="little")
                height = int.from_bytes(header[22:26], byteorder="little")
                bpp = int.from_bytes(header[28:30], byteorder="little")

                if bpp != 24:
                    print(f"BMP bpp is not 24 ({bpp}): {filename}")
                    return

                img_data = f.read()
                img_array = bytearray(img_data)

                # Swap BGR to RGB (matching C++ implementation)
                for i in range(0, len(img_array), 3):
                    temp = img_array[i]
                    img_array[i] = img_array[i + 2]
                    img_array[i + 2] = temp

                glBindTexture(GL_TEXTURE_2D, tex_id)
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
                glTexParameteri(
                    GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST
                )

                glTexImage2D(
                    GL_TEXTURE_2D,
                    0,
                    GL_RGB,
                    width,
                    height,
                    0,
                    GL_RGB,
                    GL_UNSIGNED_BYTE,
                    bytes(img_array),
                )
                gluBuild2DMipmaps(
                    GL_TEXTURE_2D,
                    3,
                    width,
                    height,
                    GL_RGB,
                    GL_UNSIGNED_BYTE,
                    bytes(img_array),
                )
                print(f"Loaded texture: {filename} ({width}x{height})")
        except FileNotFoundError:
            print(f"Texture file not found: {filename}")


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
        self.objs = OBJSETUP()
        self.keyboard = KeyboardControl()

    def loop_init(self, fileroot=None,vehicleType='car',inFarplane=1000, inWidth=800, inHeight=600):
        if fileroot is not None:
            inputroot = fileroot + f"libraries/V_{vehicleType}/"
        print("Initializing RenderControl Rendering Environment")
        objectfile = (
            f"{inputroot}Input_Files/Render.txt"
            if inputroot
            else "Input_Files/Render.txt"
        )

        try:
            lines = []
            with open(objectfile, "r") as f:
                for line in f:
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

        # Window Initialization
        glutInit()
        glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH)
        glutInitWindowSize(self.Width, self.Height)
        glutInitWindowPosition(100, 100)
        self.figure = glutCreateWindow(b"Multi Object Visualizer with Interactive Environment (MOVIE) (Running on " + os.name.encode() + b") with Python")

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

        # Setup and Load OBJs
        self.objs.Initialize(num_objects)
        if self.objs.ok:
            idx = 4
            for ii in range(num_objects):
                obj_path = (
                    f"{fileroot}{lines[idx]}" if fileroot else lines[idx]
                )
                scale_factor = float(lines[idx + 1])
                pos_scale = float(lines[idx + 2])
                self.objs.Load(obj_path, ii + 1, 1, scale_factor)
                self.state.positionscale.set(ii + 1, 1, pos_scale)
                idx += 3

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
        control_keys = ["u", "j", "h", "k", "t", "g", "m", "n"]

        if k in control_keys:
            with control_mutex:
                if k == "u":
                    self.state.control[2] -= 0.1
                elif k == "j":
                    self.state.control[2] += 0.1
                elif k == "h":
                    self.state.control[1] -= 0.1
                elif k == "k":
                    self.state.control[1] += 0.1
                elif k == "t":
                    self.state.control[0] += 0.1
                elif k == "g":
                    self.state.control[0] -= 0.1
                elif k == "m":
                    self.state.control[3] += 0.1
                elif k == "n":
                    self.state.control[3] -= 0.1

                for i in range(NUMCONTROLS):
                    if abs(self.state.control[i]) > 1.0:
                        self.state.control[i] = math.copysign(
                            1.0, self.state.control[i]
                        )

        if k == "\x1b" or k == "q":
            print("Quitting RenderControl")
            self.ok = 0
            self.ready = 0
            try:
                glutDestroyWindow(self.figure)
            except Exception:
                pass
            os._exit(0)
        elif k == "c":
            self.camera.SwitchCamera()
        elif k == "o":
            self.camera.zoom(-1)
        elif k == "i":
            self.camera.zoom(1)
        elif k == "w":
            self.camera.pitch(1)
        elif k == "s":
            self.camera.pitch(-1)
        elif k == "a":
            self.camera.yaw(-1)
        elif k == "d":
            self.camera.yaw(1)
        elif k == " ":
            print("Playing...")
            self.state.advance = 1
        elif k == "l":
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
            self.camera.pos[0],
            self.camera.pos[1],
            self.camera.pos[2],
            self.camera.target[0],
            self.camera.target[1],
            self.camera.target[2],
            self.camera.up[0],
            self.camera.up[1],
            self.camera.up[2],
        )

        # Draw all objects using compiled display lists and textures
        for ii in range(1, self.state.objects + 1):
            glBindTexture(GL_TEXTURE_2D, self.objs.texture[ii - 1])
            with state_mutex:
                cg = [self.state.cg.get(j, ii) for j in range(1, 4)]
                ptp = [self.state.ptp.get(j, ii) for j in range(1, 4)]

            glTranslatef(cg[0], cg[1], cg[2])
            glRotatef(ptp[2] * (180.0 / math.pi), 0, 0, 1)
            glRotatef(ptp[1] * (180.0 / math.pi), 0, 1, 0)
            glRotatef(ptp[0] * (180.0 / math.pi), 1, 0, 0)

            glCallList(self.objs.gl_list[ii - 1])

            glRotatef(-ptp[0] * (180.0 / math.pi), 1, 0, 0)
            glRotatef(-ptp[1] * (180.0 / math.pi), 0, 1, 0)
            glRotatef(-ptp[2] * (180.0 / math.pi), 0, 0, 1)
            glTranslatef(-cg[0], -cg[1], -cg[2])

        glutSwapBuffers()


# Global Handle Instance
glhandle_g = RenderControl()