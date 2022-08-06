#!/usr/bin/python

# This is a hasty port of the Teensy eyes code to Python...all kludgey with
# an embarrassing number of globals in the frame() function and stuff.
# Needed to get SOMETHING working, can focus on improvements next.

import argparse
import math
import pi3d
import random
import threading
import time
import RPi.GPIO as GPIO
from svg.path import Path, parse_path
from xml.dom.minidom import parse
from gfxutil import *
from snake_eyes_bonnet import SnakeEyesBonnet

# Import object detection / tracking and multi-processig
from objectDetector import *
from multiprocessing import Process
from multiprocessing import Queue

# Define custom class for eyes which overrides the multiprocessing class
class RunEyes(Process):
	# override the constructor
	def __init__(self):
		# execute the base constructor
		Process.__init__(self)
		# Set up display and member variables

		# INPUT CONFIG for eye motion ----------------------------------------------
		# ANALOG INPUTS REQUIRE SNAKE EYES BONNET

		self.JOYSTICK_X_IN   = -1    # Analog input for eye horiz pos (-1 = auto)
		self.JOYSTICK_Y_IN   = -1    # Analog input for eye vert position (")
		self.PUPIL_IN        = -1    # Analog input for pupil control (-1 = auto)
		self.JOYSTICK_X_FLIP = False # If True, reverse stick X axis
		self.JOYSTICK_Y_FLIP = False # If True, reverse stick Y axis
		self.PUPIL_IN_FLIP   = False # If True, reverse reading from PUPIL_IN
		self.TRACKING        = True  # If True, eyelid tracks pupil
		self.PUPIL_SMOOTH    = 16    # If > 0, filter input from PUPIL_IN
		self.PUPIL_MIN       = 0.0   # Lower analog range from PUPIL_IN
		self.PUPIL_MAX       = 1.0   # Upper "
		self.WINK_L_PIN      = 22    # GPIO pin for LEFT eye wink button
		self.BLINK_PIN       = 23    # GPIO pin for blink button (BOTH eyes)
		self.WINK_R_PIN      = 24    # GPIO pin for RIGHT eye wink button
		self.AUTOBLINK       = True  # If True, eyes blink autonomously
		self.CRAZY_EYES      = False # If True, each eye moves in different directions


		# GPIO initialization ------------------------------------------------------

		GPIO.setmode(GPIO.BCM)
		if self.WINK_L_PIN >= 0: GPIO.setup(self.WINK_L_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
		if self.BLINK_PIN  >= 0: GPIO.setup(self.BLINK_PIN , GPIO.IN, pull_up_down=GPIO.PUD_UP)
		if self.WINK_R_PIN >= 0: GPIO.setup(self.WINK_R_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)


		# ADC stuff ----------------------------------------------------------------

		# ADC channels are read and stored in a separate thread to avoid slowdown
		# from blocking operations. The animation loop can read at its leisure.

		if self.JOYSTICK_X_IN >= 0 or self.JOYSTICK_Y_IN >= 0 or self.PUPIL_IN >= 0:
			self.bonnet = SnakeEyesBonnet(daemon=True)
			self.bonnet.setup_channel(self.JOYSTICK_X_IN, reverse=self.JOYSTICK_X_FLIP)
			self.bonnet.setup_channel(self.JOYSTICK_Y_IN, reverse=self.JOYSTICK_Y_FLIP)
			self.bonnet.setup_channel(self.PUPIL_IN, reverse=self.PUPIL_IN_FLIP)
			self.bonnet.start()


		# Load SVG file, extract paths & convert to point lists --------------------

		self.dom               = parse("graphics/eye.svg")
		self.vb                = get_view_box(self.dom)
		self.pupilMinPts       = get_points(self.dom, "pupilMin"      , 32, True , True )
		self.pupilMaxPts       = get_points(self.dom, "pupilMax"      , 32, True , True )
		self.irisPts           = get_points(self.dom, "iris"          , 32, True , True )
		self.scleraFrontPts    = get_points(self.dom, "scleraFront"   ,  0, False, False)
		self.scleraBackPts     = get_points(self.dom, "scleraBack"    ,  0, False, False)
		self.upperLidClosedPts = get_points(self.dom, "upperLidClosed", 33, False, True )
		self.upperLidOpenPts   = get_points(self.dom, "upperLidOpen"  , 33, False, True )
		self.upperLidEdgePts   = get_points(self.dom, "upperLidEdge"  , 33, False, False)
		self.lowerLidClosedPts = get_points(self.dom, "lowerLidClosed", 33, False, False)
		self.lowerLidOpenPts   = get_points(self.dom, "lowerLidOpen"  , 33, False, False)
		self.lowerLidEdgePts   = get_points(self.dom, "lowerLidEdge"  , 33, False, False)


		# Set up display and initialize pi3d ---------------------------------------

		self.DISPLAY = pi3d.Display.create(samples=4)
		self.DISPLAY.set_background(0, 0, 0, 1) # r,g,b,alpha

		# eyeRadius is the size, in pixels, at which the whole eye will be rendered
		# onscreen.  eyePosition, also pixels, is the offset (left or right) from
		# the center point of the screen to the center of each eye.  This geometry
		# is explained more in-depth in fbx2.c.
		self.eyePosition = self.DISPLAY.width / 4
		self.eyeRadius   = 128  # Default; use 240 for IPS screens

		self.parser = argparse.ArgumentParser()
		self.parser.add_argument("--radius", type=int)
		args, _ = self.parser.parse_known_args()
		if args.radius:
			self.eyeRadius = args.radius


		# A 2D camera is used, mostly to allow for pixel-accurate eye placement,
		# but also because perspective isn't really helpful or needed here, and
		# also this allows eyelids to be handled somewhat easily as 2D planes.
		# Line of sight is down Z axis, allowing conventional X/Y cartesion
		# coords for 2D positions.
		self.cam    = pi3d.Camera(is_3d=False, at=(0,0,0), eye=(0,0,-1000))
		self.shader = pi3d.Shader("uv_light")
		self.light  = pi3d.Light(lightpos=(0, -500, -500), lightamb=(0.2, 0.2, 0.2))


		# Load texture maps --------------------------------------------------------

		self.irisMap   = pi3d.Texture("graphics/iris.jpg"  , mipmap=False,
					  filter=pi3d.constants.GL_LINEAR)
		self.scleraMap = pi3d.Texture("graphics/sclera.png", mipmap=False,
					  filter=pi3d.constants.GL_LINEAR, blend=True)
		self.lidMap    = pi3d.Texture("graphics/lid.png"   , mipmap=False,
					  filter=pi3d.constants.GL_LINEAR, blend=True)
		# U/V map may be useful for debugging texture placement; not normally used
		#uvMap     = pi3d.Texture("graphics/uv.png"    , mipmap=False,
		#              filter=pi3d.constants.GL_LINEAR, blend=False, m_repeat=True)


		# Initialize static geometry -----------------------------------------------

		# Transform point lists to eye dimensions
		scale_points(self.pupilMinPts      , self.vb, self.eyeRadius)
		scale_points(self.pupilMaxPts      , self.vb, self.eyeRadius)
		scale_points(self.irisPts          , self.vb, self.eyeRadius)
		scale_points(self.scleraFrontPts   , self.vb, self.eyeRadius)
		scale_points(self.scleraBackPts    , self.vb, self.eyeRadius)
		scale_points(self.upperLidClosedPts, self.vb, self.eyeRadius)
		scale_points(self.upperLidOpenPts  , self.vb, self.eyeRadius)
		scale_points(self.upperLidEdgePts  , self.vb, self.eyeRadius)
		scale_points(self.lowerLidClosedPts, self.vb, self.eyeRadius)
		scale_points(self.lowerLidOpenPts  , self.vb, self.eyeRadius)
		scale_points(self.lowerLidEdgePts  , self.vb, self.eyeRadius)

		# Regenerating flexible object geometry (such as eyelids during blinks, or
		# iris during pupil dilation) is CPU intensive, can noticably slow things
		# down, especially on single-core boards.  To reduce this load somewhat,
		# determine a size change threshold below which regeneration will not occur;
		# roughly equal to 1/4 pixel, since 4x4 area sampling is used.

		# Determine change in pupil size to trigger iris geometry regen
		self.irisRegenThreshold = 0.0
		self.a = points_bounds(self.pupilMinPts) # Bounds of pupil at min size (in pixels)
		self.b = points_bounds(self.pupilMaxPts) # " at max size
		self.maxDist = max(abs(self.a[0] - self.b[0]), abs(self.a[1] - self.b[1]), # Determine distance of max
					  abs(self.a[2] - self.b[2]), abs(self.a[3] - self.b[3])) # variance around each edge
		# maxDist is motion range in pixels as pupil scales between 0.0 and 1.0.
		# 1.0 / maxDist is one pixel's worth of scale range.  Need 1/4 that...
		if self.maxDist > 0: self.irisRegenThreshold = 0.25 / self.maxDist

		# Determine change in eyelid values needed to trigger geometry regen.
		# This is done a little differently than the pupils...instead of bounds,
		# the distance between the middle points of the open and closed eyelid
		# paths is evaluated, then similar 1/4 pixel threshold is determined.
		self.upperLidRegenThreshold = 0.0
		self.lowerLidRegenThreshold = 0.0
		self.p1 = self.upperLidOpenPts[len(self.upperLidOpenPts) // 2]
		self.p2 = self.upperLidClosedPts[len(self.upperLidClosedPts) // 2]
		self.dx = self.p2[0] - self.p1[0]
		self.dy = self.p2[1] - self.p1[1]
		self.d  = self.dx * self.dx + self.dy * self.dy
		if self.d > 0: self.upperLidRegenThreshold = 0.25 / math.sqrt(self.d)
		self.p1 = self.lowerLidOpenPts[len(self.lowerLidOpenPts) // 2]
		self.p2 = self.lowerLidClosedPts[len(self.lowerLidClosedPts) // 2]
		self.dx = self.p2[0] - self.p1[0]
		self.dy = self.p2[1] - self.p1[1]
		self.d  = self.dx * self.dx + self.dy * self.dy
		if self.d > 0: self.lowerLidRegenThreshold = 0.25 / math.sqrt(self.d)

		# Generate initial iris meshes; vertex elements will get replaced on
		# a per-frame basis in the main loop, this just sets up textures, etc.
		self.rightIris = mesh_init((32, 4), (0, 0.5 / self.irisMap.iy), True, False)
		self.rightIris.set_textures([self.irisMap])
		self.rightIris.set_shader(self.shader)
		# Left iris map U value is offset by 0.5; effectively a 180 degree
		# rotation, so it's less obvious that the same texture is in use on both.
		self.leftIris = mesh_init((32, 4), (0.5, 0.5 / self.irisMap.iy), True, False)
		self.leftIris.set_textures([self.irisMap])
		self.leftIris.set_shader(self.shader)
		self.irisZ = zangle(self.irisPts, self.eyeRadius)[0] * 0.99 # Get iris Z depth, for later

		# Eyelid meshes are likewise temporary; texture coordinates are
		# assigned here but geometry is dynamically regenerated in main loop.
		self.leftUpperEyelid = mesh_init((33, 5), (0, 0.5 / self.lidMap.iy), False, True)
		self.leftUpperEyelid.set_textures([self.lidMap])
		self.leftUpperEyelid.set_shader(self.shader)
		self.leftLowerEyelid = mesh_init((33, 5), (0, 0.5 / self.lidMap.iy), False, True)
		self.leftLowerEyelid.set_textures([self.lidMap])
		self.leftLowerEyelid.set_shader(self.shader)

		self.rightUpperEyelid = mesh_init((33, 5), (0, 0.5 / self.lidMap.iy), False, True)
		self.rightUpperEyelid.set_textures([self.lidMap])
		self.rightUpperEyelid.set_shader(self.shader)
		self.rightLowerEyelid = mesh_init((33, 5), (0, 0.5 / self.lidMap.iy), False, True)
		self.rightLowerEyelid.set_textures([self.lidMap])
		self.rightLowerEyelid.set_shader(self.shader)

		# Generate scleras for each eye...start with a 2D shape for lathing...
		self.angle1 = zangle(self.scleraFrontPts, self.eyeRadius)[1] # Sclera front angle
		self.angle2 = zangle(self.scleraBackPts , self.eyeRadius)[1] # " back angle
		self.aRange = 180 - self.angle1 - self.angle2
		self.pts    = []

		# ADD EXTRA INITIAL POINT because of some weird behavior with Pi3D and
		# VideoCore VI with the Lathed shapes we make later. This adds a *tiny*
		# ring of extra polygons that simply disappear on screen. It's not
		# necessary on VC4, but not harmful either, so we just do it rather
		# than try to be all clever.
		self.ca, self.sa = pi3d.Utility.from_polar((90 - self.angle1) + self.aRange * 0.0001)
		self.pts.append((self.ca * self.eyeRadius, self.sa * self.eyeRadius))

		for i in range(24):
			self.ca, self.sa = pi3d.Utility.from_polar((90 - self.angle1) - self.aRange * i / 23)
			self.pts.append((self.ca * self.eyeRadius, self.sa * self.eyeRadius))

		# Scleras are generated independently (object isn't re-used) so each
		# may have a different image map (heterochromia, corneal scar, or the
		# same image map can be offset on one so the repetition isn't obvious).
		self.leftEye = pi3d.Lathe(path=self.pts, sides=64)
		self.leftEye.set_textures([self.scleraMap])
		self.leftEye.set_shader(self.shader)
		re_axis(self.leftEye, 0)
		self.rightEye = pi3d.Lathe(path=self.pts, sides=64)
		self.rightEye.set_textures([self.scleraMap])
		self.rightEye.set_shader(self.shader)
		re_axis(self.rightEye, 0.5) # Image map offset = 180 degree rotation


		# Init global stuff --------------------------------------------------------

		self.mykeys = pi3d.Keyboard() # For capturing key presses

		self.startX       = 0.0 #random.uniform(-30.0, 30.0)
		self.n            = math.sqrt(900.0 - self.startX * self.startX)
		self.startY       = 0.0 #random.uniform(-n, n)
		self.destX        = self.startX
		self.destY        = self.startY
		self.curX         = self.startX
		self.curY         = self.startY
		self.moveDuration = random.uniform(0.075, 0.175)
		self.holdDuration = 5.0 #random.uniform(0.1, 1.1)
		self.startTime    = 0.0
		self.isMoving     = False

		self.startXR      = random.uniform(-30.0, 30.0)
		self.n            = math.sqrt(900.0 - self.startX * self.startX)
		self.startYR      = random.uniform(-self.n, self.n)
		self.destXR       = self.startXR
		self.destYR       = self.startYR
		self.curXR        = self.startXR
		self.curYR        = self.startYR
		self.moveDurationR = random.uniform(0.075, 0.175)
		self.holdDurationR = random.uniform(0.1, 1.1)
		self.startTimeR    = 0.0
		self.isMovingR     = False

		self.frames        = 0
		self.beginningTime = time.time()

		self.rightEye.positionX(-self.eyePosition)
		self.rightIris.positionX(-self.eyePosition)
		self.rightUpperEyelid.positionX(-self.eyePosition)
		self.rightUpperEyelid.positionZ(-self.veyeRadius - 42)
		self.rightLowerEyelid.positionX(-self.eyePosition)
		self.rightLowerEyelid.positionZ(-self.eyeRadius - 42)

		self.leftEye.positionX(self.eyePosition)
		self.leftIris.positionX(self.eyePosition)
		self.leftUpperEyelid.positionX(self.eyePosition)
		self.leftUpperEyelid.positionZ(-self.eyeRadius - 42)
		self.leftLowerEyelid.positionX(self.eyePosition)
		self.leftLowerEyelid.positionZ(-self.eyeRadius - 42)

		self.currentPupilScale       =  0.5
		self.prevPupilScale          = -1.0 # Force regen on first frame
		self.prevLeftUpperLidWeight  = 0.5
		self.prevLeftLowerLidWeight  = 0.5
		self.prevRightUpperLidWeight = 0.5
		self.prevRightLowerLidWeight = 0.5
		self.prevLeftUpperLidPts  = points_interp(self.upperLidOpenPts, self.upperLidClosedPts, 0.5)
		self.prevLeftLowerLidPts  = points_interp(self.lowerLidOpenPts, self.lowerLidClosedPts, 0.5)
		self.prevRightUpperLidPts = points_interp(self.upperLidOpenPts, self.upperLidClosedPts, 0.5)
		self.prevRightLowerLidPts = points_interp(self.lowerLidOpenPts, self.lowerLidClosedPts, 0.5)

		self.luRegen = True
		self.llRegen = True
		self.ruRegen = True
		self.rlRegen = True

		self.timeOfLastBlink = 0.0
		self.timeToNextBlink = 1.0
		# These are per-eye (left, right) to allow winking:
		self.blinkStateLeft      = 0 # NOBLINK
		self.blinkStateRight     = 0
		self.blinkDurationLeft   = 0.1
		self.blinkDurationRight  = 0.1
		self.blinkStartTimeLeft  = 0
		self.blinkStartTimeRight = 0

		self.trackingPos = 0.3
		self.trackingPosR = 0.3


		# Generate one frame of imagery
		def frame(p):

			# global startX, startY, destX, destY, curX, curY
			# global startXR, startYR, destXR, destYR, curXR, curYR
			# global moveDuration, holdDuration, startTime, isMoving
			# global moveDurationR, holdDurationR, startTimeR, isMovingR
			# global frames
			# global leftIris, rightIris
			# global pupilMinPts, pupilMaxPts, irisPts, irisZ
			# global leftEye, rightEye
			# global leftUpperEyelid, leftLowerEyelid, rightUpperEyelid, rightLowerEyelid
			# global upperLidOpenPts, upperLidClosedPts, lowerLidOpenPts, lowerLidClosedPts
			# global upperLidEdgePts, lowerLidEdgePts
			# global prevLeftUpperLidPts, prevLeftLowerLidPts, prevRightUpperLidPts, prevRightLowerLidPts
			# global leftUpperEyelid, leftLowerEyelid, rightUpperEyelid, rightLowerEyelid
			# global prevLeftUpperLidWeight, prevLeftLowerLidWeight, prevRightUpperLidWeight, prevRightLowerLidWeight
			# global prevPupilScale
			# global irisRegenThreshold, upperLidRegenThreshold, lowerLidRegenThreshold
			# global luRegen, llRegen, ruRegen, rlRegen
			# global timeOfLastBlink, timeToNextBlink
			# global blinkStateLeft, blinkStateRight
			# global blinkDurationLeft, blinkDurationRight
			# global blinkStartTimeLeft, blinkStartTimeRight
			# global trackingPos
			# global trackingPosR

			self.DISPLAY.loop_running()

			now = time.time()
			dt  = now - self.startTime
			dtR  = now - self.startTimeR

			self.frames += 1
		#	if(now > beginningTime):
		#		print(frames/(now-beginningTime))

			if self.JOYSTICK_X_IN >= 0 and self.JOYSTICK_Y_IN >= 0:
				# Eye position from analog inputs

				self.curX = self.bonnet.channel[self.JOYSTICK_X_IN].value
				self.curY = self.bonnet.channel[self.JOYSTICK_Y_IN].value
				self.curX = -30.0 + self.curX * 60.0
				self.curY = -30.0 + self.curY * 60.0
			else :
				# Autonomous eye position
				if self.isMoving == True:
					if dt <= self.moveDuration:
						scale        = (now - self.startTime) / self.moveDuration
						# Ease in/out curve: 3*t^2-2*t^3
						scale = 3.0 * scale * scale - 2.0 * scale * scale * scale
						self.curX         = self.startX + (self.destX - self.startX) * scale
						self.curY         = self.startY + (self.destY - self.startY) * scale
					else:
						self.startX       = self.destX
						self.startY       = self.destY
						self.curX         = self.destX
						self.curY         = self.destY
						self.holdDuration = .3 #random.uniform(0.1, 1.1)
						self.startTime    = now
						self.isMoving     = False
				else:
					if dt >= self.holdDuration:
						self.destX        = random.uniform(-30.0, 30.0)
						self.n            = math.sqrt(900.0 - self.destX * self.destX)
						self.destY        = random.uniform(-self.n, self.n)
						self.moveDuration = 0.175 #random.uniform(0.075, 0.175)
						self.startTime    = now
						self.isMoving     = True

				# repeat for other eye if CRAZY_EYES
			if self.CRAZY_EYES:
					if self.isMovingR == True:
						if dtR <= self.moveDurationR:
							scale        = (now - self.startTimeR) / self.moveDurationR
							# Ease in/out curve: 3*t^2-2*t^3
							scale = 3.0 * scale * scale - 2.0 * scale * scale * scale
							self.curXR        = self.startXR + (self.destXR - self.startXR) * scale
							self.curYR        = self.startYR + (self.destYR - self.startYR) * scale
						else:
							self.startXR      = self.destXR
							self.startYR      = self.destYR
							self.curXR        = self.destXR
							self.curYR        = self.destYR
							self.holdDurationR = random.uniform(0.1, 1.1)
							self.startTimeR    = now
							self.isMovingR     = False
					else:
						if dtR >= self.holdDurationR:
							self.destXR        = random.uniform(-30.0, 30.0)
							self.n             = math.sqrt(900.0 - self.destXR * self.destXR)
							self.destYR        = random.uniform(-self.n, self.n)
							self.moveDurationR = random.uniform(0.075, 0.175)
							self.startTimeR    = now
							self.isMovingR     = True

			# Regenerate iris geometry only if size changed by >= 1/4 pixel
			if abs(p - self.prevPupilScale) >= self.irisRegenThreshold:
				# Interpolate points between min and max pupil sizes
				self.interPupil = points_interp(self.pupilMinPts, self.pupilMaxPts, p)
				# Generate mesh between interpolated pupil and iris bounds
				mesh = points_mesh((None, self.interPupil, self.irisPts), 4, -self.irisZ, True)
				# Assign to both eyes
				self.leftIris.re_init(pts=mesh)
				self.rightIris.re_init(pts=mesh)
				self.prevPupilScale = p

			# Eyelid WIP

			if self.AUTOBLINK and (now - self.timeOfLastBlink) >= self.timeToNextBlink:
				self.timeOfLastBlink = now
				self.duration        = random.uniform(0.035, 0.06)
				if self.blinkStateLeft != 1:
					self.blinkStateLeft     = 1 # ENBLINK
					self.blinkStartTimeLeft = now
					self.blinkDurationLeft  = self.duration
				if self.blinkStateRight != 1:
					self.blinkStateRight     = 1 # ENBLINK
					self.blinkStartTimeRight = now
					self.blinkDurationRight  = self.duration
				self.timeToNextBlink = self.duration * 3 + random.uniform(0.0, 4.0)

			if self.blinkStateLeft: # Left eye currently winking/blinking?
				# Check if blink time has elapsed...
				if (now - self.blinkStartTimeLeft) >= self.blinkDurationLeft:
					# Yes...increment blink state, unless...
					if (self.blinkStateLeft == 1 and # Enblinking and...
						((self.BLINK_PIN >= 0 and    # blink pin held, or...
						  GPIO.input(self.BLINK_PIN) == GPIO.LOW) or
						(self.WINK_L_PIN >= 0 and    # wink pin held
						  GPIO.input(self.WINK_L_PIN) == GPIO.LOW))):
						# Don't advance yet; eye is held closed
						pass
					else:
						self.blinkStateLeft += 1
						if self.blinkStateLeft > 2:
							self.blinkStateLeft = 0 # NOBLINK
						else:
							self.blinkDurationLeft *= 2.0
							self.blinkStartTimeLeft = now
			else:
				if self.WINK_L_PIN >= 0 and GPIO.input(self.WINK_L_PIN) == GPIO.LOW:
					self.blinkStateLeft     = 1 # ENBLINK
					self.blinkStartTimeLeft = now
					self.blinkDurationLeft  = random.uniform(0.035, 0.06)

			if self.blinkStateRight: # Right eye currently winking/blinking?
				# Check if blink time has elapsed...
				if (now - self.blinkStartTimeRight) >= self.blinkDurationRight:
					# Yes...increment blink state, unless...
					if (self.blinkStateRight == 1 and # Enblinking and...
						((self.BLINK_PIN >= 0 and    # blink pin held, or...
						  GPIO.input(self.BLINK_PIN) == GPIO.LOW) or
						(self.WINK_R_PIN >= 0 and    # wink pin held
						  GPIO.input(self.WINK_R_PIN) == GPIO.LOW))):
						# Don't advance yet; eye is held closed
						pass
					else:
						self.blinkStateRight += 1
						if self.blinkStateRight > 2:
							self.blinkStateRight = 0 # NOBLINK
						else:
							self.blinkDurationRight *= 2.0
							self.blinkStartTimeRight = now
			else:
				if self.WINK_R_PIN >= 0 and GPIO.input(self.WINK_R_PIN) == GPIO.LOW:
					self.blinkStateRight     = 1 # ENBLINK
					self.blinkStartTimeRight = now
					self.blinkDurationRight  = random.uniform(0.035, 0.06)

			if self.BLINK_PIN >= 0 and GPIO.input(self.BLINK_PIN) == GPIO.LOW:
				self.duration = random.uniform(0.035, 0.06)
				if self.blinkStateLeft == 0:
					self.blinkStateLeft     = 1
					self.blinkStartTimeLeft = now
					self.blinkDurationLeft  = self.duration
				if self.blinkStateRight == 0:
					self.blinkStateRight     = 1
					self.blinkStartTimeRight = now
					self.blinkDurationRight  = self.duration

			if self.TRACKING:
				self.n = 0.4 - self.curY / 60.0
				if   self.n < 0.0: self.n = 0.0
				elif self.n > 1.0: self.n = 1.0
				self.trackingPos = (self.trackingPos * 3.0 + self.n) * 0.25
				if self.CRAZY_EYES:
					self.n = 0.4 - self.curYR / 60.0
					if   self.n < 0.0: self.n = 0.0
					elif self.n > 1.0: self.n = 1.0
					self.trackingPosR = (self.trackingPosR * 3.0 + self.n) * 0.25

			if self.blinkStateLeft:
				self.n = (now - self.blinkStartTimeLeft) / self.blinkDurationLeft
				if self.n > 1.0: n = 1.0
				if self.blinkStateLeft == 2: self.n = 1.0 - self.n
			else:
				self.n = 0.0
			newLeftUpperLidWeight = self.trackingPos + (n * (1.0 - self.trackingPos))
			newLeftLowerLidWeight = (1.0 - self.trackingPos) + (n * self.trackingPos)

			if self.blinkStateRight:
				self.n = (now - self.blinkStartTimeRight) / self.blinkDurationRight
				if self.n > 1.0: self.n = 1.0
				if self.blinkStateRight == 2: self.n = 1.0 - self.n
			else:
				self.n = 0.0
			if self.CRAZY_EYES:
				newRightUpperLidWeight = self.trackingPosR + (self.n * (1.0 - self.trackingPosR))
				newRightLowerLidWeight = (1.0 - self.trackingPosR) + (self.n * self.trackingPosR)
			else:
				newRightUpperLidWeight = self.trackingPos + (self.n * (1.0 - self.trackingPos))
				newRightLowerLidWeight = (1.0 - self.trackingPos) + (self.n * self.trackingPos)

			if (self.luRegen or (abs(newLeftUpperLidWeight - self.prevLeftUpperLidWeight) >=
			  self.upperLidRegenThreshold)):
				newLeftUpperLidPts = points_interp(self.upperLidOpenPts,
				  self.upperLidClosedPts, newLeftUpperLidWeight)
				if newLeftUpperLidWeight > self.prevLeftUpperLidWeight:
					self.leftUpperEyelid.re_init(pts=points_mesh(
					  (self.upperLidEdgePts, self.prevLeftUpperLidPts,
					  newLeftUpperLidPts), 5, 0, False))
				else:
					self.leftUpperEyelid.re_init(pts=points_mesh(
					  (self.upperLidEdgePts, newLeftUpperLidPts,
					  self.prevLeftUpperLidPts), 5, 0, False))
				self.prevLeftUpperLidPts    = newLeftUpperLidPts
				self.prevLeftUpperLidWeight = newLeftUpperLidWeight
				self.luRegen = True
			else:
				self.luRegen = False

			if (self.llRegen or (abs(newLeftLowerLidWeight - self.prevLeftLowerLidWeight) >=
			  self.lowerLidRegenThreshold)):
				newLeftLowerLidPts = points_interp(self.lowerLidOpenPts,
				  self.lowerLidClosedPts, newLeftLowerLidWeight)
				if newLeftLowerLidWeight > self.prevLeftLowerLidWeight:
					self.leftLowerEyelid.re_init(pts=points_mesh(
					  (self.lowerLidEdgePts, self.prevLeftLowerLidPts,
					  newLeftLowerLidPts), 5, 0, False))
				else:
					self.leftLowerEyelid.re_init(pts=points_mesh(
					  (self.lowerLidEdgePts, newLeftLowerLidPts,
					  self.prevLeftLowerLidPts), 5, 0, False))
				self.prevLeftLowerLidWeight = newLeftLowerLidWeight
				self.prevLeftLowerLidPts    = newLeftLowerLidPts
				self.llRegen = True
			else:
				self.llRegen = False

			if (self.ruRegen or (abs(newRightUpperLidWeight - self.prevRightUpperLidWeight) >=
			  self.upperLidRegenThreshold)):
				newRightUpperLidPts = points_interp(self.upperLidOpenPts,
				  self.upperLidClosedPts, newRightUpperLidWeight)
				if newRightUpperLidWeight > self.prevRightUpperLidWeight:
					self.rightUpperEyelid.re_init(pts=points_mesh(
					  (self.upperLidEdgePts, self.prevRightUpperLidPts,
					  newRightUpperLidPts), 5, 0, True))
				else:
					self.rightUpperEyelid.re_init(pts=points_mesh(
					  (self.upperLidEdgePts, newRightUpperLidPts,
					  self.prevRightUpperLidPts), 5, 0, True))
				self.prevRightUpperLidWeight = newRightUpperLidWeight
				self.prevRightUpperLidPts    = newRightUpperLidPts
				self.ruRegen = True
			else:
				self.ruRegen = False

			if (self.rlRegen or (abs(newRightLowerLidWeight - self.prevRightLowerLidWeight) >=
			  self.lowerLidRegenThreshold)):
				newRightLowerLidPts = points_interp(self.lowerLidOpenPts,
				  self.lowerLidClosedPts, newRightLowerLidWeight)
				if newRightLowerLidWeight > self.prevRightLowerLidWeight:
					self.rightLowerEyelid.re_init(pts=points_mesh(
					  (self.lowerLidEdgePts, self.prevRightLowerLidPts,
					  newRightLowerLidPts), 5, 0, True))
				else:
					self.rightLowerEyelid.re_init(pts=points_mesh(
					  (self.lowerLidEdgePts, newRightLowerLidPts,
					  self.prevRightLowerLidPts), 5, 0, True))
				self.prevRightLowerLidWeight = newRightLowerLidWeight
				self.prevRightLowerLidPts    = newRightLowerLidPts
				self.rlRegen = True
			else:
				self.rlRegen = False

			convergence = 2.0

			# Right eye (on screen left)
			if self.CRAZY_EYES:
				self.rightIris.rotateToX(self.curYR)
				self.rightIris.rotateToY(self.curXR - convergence)
				self.rightIris.draw()
				self.rightEye.rotateToX(self.curYR)
				self.rightEye.rotateToY(self.curXR - convergence)
			else:
				self.rightIris.rotateToX(self.curY)
				self.rightIris.rotateToY(self.curX - convergence)
				self.rightIris.draw()
				self.rightEye.rotateToX(self.curY)
				self.rightEye.rotateToY(self.curX - convergence)
			self.rightEye.draw()

			# Left eye (on screen right)

			self.leftIris.rotateToX(self.curY)
			self.leftIris.rotateToY(self.curX + convergence)
			self.leftIris.draw()
			self.leftEye.rotateToX(self.curY)
			self.leftEye.rotateToY(self.curX + convergence)
			self.leftEye.draw()

			self.leftUpperEyelid.draw()
			self.leftLowerEyelid.draw()
			self.rightUpperEyelid.draw()
			self.rightLowerEyelid.draw()

			k = self.mykeys.read()
			if k==27:
				self.mykeys.close()
				self.DISPLAY.stop()
				exit(0)


		def split( # Recursive simulated pupil response when no analog sensor
		  startValue, # Pupil scale starting value (0.0 to 1.0)
		  endValue,   # Pupil scale ending value (")
		  duration,   # Start-to-end time, floating-point seconds
		  range):     # +/- random pupil scale at midpoint
			self.startTime = time.time()
			if range >= 0.125: # Limit subdvision count, because recursion
				self.duration *= 0.5 # Split time & range in half for subdivision,
				range    *= 0.5 # then pick random center point within range:
				self.midValue  = ((self.startValue + self.endValue - range) * 0.5 +
							 random.uniform(0.0, range))
				split(self.startValue, self.midValue, self.duration, range)
				split(self.midValue  , self.endValue, self.duration, range)
			else: # No more subdivisons, do iris motion...
				dv = self.endValue - self.startValue
				while True:
					dt = time.time() - self.startTime
					if dt >= self.duration: break
					v = self.startValue + dv * dt / self.duration
					if   v < self.PUPIL_MIN: v = self.PUPIL_MIN
					elif v > self.PUPIL_MAX: v = self.PUPIL_MAX
					frame(v) # Draw frame w/interim pupil scale value


		def Run():
			# global currentPupilScale
			
			while True:
				
				if self.PUPIL_IN >= 0: # Pupil scale from sensor
					v = self.bonnet.channel[self.PUPIL_IN].value
					# If you need to calibrate PUPIL_MIN and MAX,
					# add a 'print v' here for testing.
					if   v < self.PUPIL_MIN: v = self.PUPIL_MIN
					elif v > self.PUPIL_MAX: v = self.PUPIL_MAX
					# Scale to 0.0 to 1.0:
					v = (v - self.PUPIL_MIN) / (self.PUPIL_MAX - self.PUPIL_MIN)
					if self.PUPIL_SMOOTH > 0:
						v = ((self.currentPupilScale * (self.PUPIL_SMOOTH - 1) + v) /
							 self.PUPIL_SMOOTH)
					frame(v)   # Pass in object location data
				else: # Fractal auto pupil scale
					v = random.random()
					split(self.currentPupilScale, v, 4.0, 1.0)
				self.currentPupilScale = v

# MAIN LOOP -- runs continuously -------------------------------------------

Eyes = RunEyes()

RunEyes.Run()

# Create queue to share object position data between processes
#queue = Queue()

# start the consumer
#consumer_process = Process(target=runEyes)
#consumer_process.start()

# start the producer
#producer_process = Process(target=ObjectTracker, args=(queue,))
#producer_process.start()

# wait for all processes to finish
#producer_process.join()
#consumer_process.join()








