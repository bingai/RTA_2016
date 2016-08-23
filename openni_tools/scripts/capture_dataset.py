#!/usr/bin/python

import os, sys
import subprocess
import shutil
import fnmatch
import curses

################################################################################
# Print help
################################################################################
def printUsage(argv0):

  [curDir, scriptFilename] = os.path.split(argv0)

  print "Record RGB-D sequences to oni files. Each oni file is written to a " \
        "subfolder in the output directory. Subfolders are numbered as scene_x"
  print "   \dataset_dir"
  print "     \scene_000"
  print "     \scene_001"
  print "     ..."
  print
  print "Usage: '" + scriptFilename + "' <output_dir>"

################################################################################
# Parse command line
################################################################################
def parseCommandLine(argv):

  # Find help flag
  for arg in argv:
    if arg == "-h" or arg == "--help":
      printUsage(argv[0])
      sys.exit(-1)

  # If there are no arguments - complain
  if len(argv) < 2:
    print "You must provide output directory"
    sys.exit(-1)

  # Otherwise return first argument
  return argv[1]

################################################################################
# Initialize curses terminal
################################################################################
def cursesInit():
  stdscr = curses.initscr() 
  curses.cbreak() 
  stdscr.keypad(1)
  curses.curs_set(0)

  return stdscr

################################################################################
# Return terminal to original state
################################################################################
def cursesDeinit(stdscr):
  curses.nocbreak()
  stdscr.keypad(0)
  curses.curs_set(1)
  curses.echo()
  curses.endwin()

################################################################################
# Clear lines
################################################################################
def cursesClearLines(stdscr, start_line, end_line):

  # Check input parameters
  [maxY, maxX] = stdscr.getmaxyx()
  start_line  = max(start_line, 0)
  end_line    = min(end_line, maxY-1)

  if start_line >= end_line:
    return

  # Save current cursor location
  y = 0
  x = 0
  [y, x] = stdscr.getyx()

  # Clear lines
  for id in range(start_line, end_line):
    stdscr.move(id,0)
    stdscr.clrtoeol()  

  # Move cursor back
  stdscr.move(y,x)

################################################################################
# Get a key press
################################################################################
def getQuit(stdscr):

  # Wait for keypress
  quit = False
  keyPressed = False
  key = ''
  while not keyPressed:
    key = stdscr.getch()
    # stdscr.addstr(2,0,str(int(key))) 

    if key == ord('q') or key == 27:
      keyPressed = True
      quit = True

    elif key == 32:
      keyPressed = True

  return quit

################################################################################
# Main
################################################################################
def main(argv):

  # Parse comman line
  outputDir = parseCommandLine(argv)
  outputDir = os.path.abspath(outputDir)

  # Check that output directory exists
  if not os.path.exists(outputDir):
    print "Output directory '" + outputDir + "' does not exist"
    print "Aborting..."
    sys.exit()

  #-----------------------------------------------------------------------------
  # Check that oni recording script exists
  #-----------------------------------------------------------------------------

  recordScriptFilename = "../bin/openni_capture"

  if not os.path.exists(recordScriptFilename):
    print "Oni record script could not be found '" + recordScriptFilename + "'"
    print "Aborting..."
    return -1

  #-----------------------------------------------------------------------------
  # Check for existing scenes
  #-----------------------------------------------------------------------------

  stdscr = cursesInit()

  existingSceneIds = []
  maxEcistingSceneId = 0
  startSceneId = 0

  for file in os.listdir(outputDir):
    if fnmatch.fnmatch(file, 'scene_???'):
      numberString = file.split('_')
      numberString = numberString[1]

      if not numberString.isdigit():
        continue

      existingSceneIds.append(int(numberString))

  if len(existingSceneIds) > 0:
    maxEcistingSceneId = max(existingSceneIds)
    startSceneId = maxEcistingSceneId + 1
  else:
    startSceneId = 0

  stdscr.addstr(0, 0, "Writing scenes to:")
  stdscr.addstr(1, 2, outputDir)
  line = "Starting new scene numbering at " + str(startSceneId)
  stdscr.addstr(3, 0, line)

  #-----------------------------------------------------------------------------
  # Run capture in loop 
  #-----------------------------------------------------------------------------

  numScenesCaptured = 0
  captureSuccess = False
  quit = False

  while not quit:

    # Clear lines
    cursesClearLines(stdscr, 5, 40)

    # Find current output directory
    sceneIdStr = "%03d" % (startSceneId + numScenesCaptured)
    sceneStr = "scene_" + sceneIdStr
    curOutputDir = os.path.join(outputDir, sceneStr)
    line = "Capturing '" + sceneStr + "'"
    stdscr.addstr(5,0,line)
    stdscr.refresh()

    # Create directory if needed
    if not os.path.exists(curOutputDir):
      os.makedirs(curOutputDir)

    # Capture scene
    curOutputFile = os.path.join(curOutputDir, sceneStr + ".oni")
    cmd = [ recordScriptFilename, curOutputFile]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)


    # Check if capture was succeessfull
    captureSuccess = p.wait() == 0

    if captureSuccess:
      if os.path.isfile(curOutputFile):

        stdscr.addstr(6,0,"Success!")
        stdscr.addstr(7, 0, "Press 'Esc' or 'q' to quit. Press 'Space' to capture next sequence") 
        numScenesCaptured = numScenesCaptured + 1

      else:

        stdscr.addstr(6,0,"oni file was not captured")
        stdscr.addstr(7, 0, "Press 'Esc' or 'q' to quit. Press 'Space' to capture next sequence")
        shutil.rmtree(curOutputDir)
        
      stdscr.refresh()
      quit = getQuit(stdscr)

      if quit:
        line = str(numScenesCaptured) + " scenes captured"
        stdscr.addstr(9,0,line)
        stdscr.addstr(10,0,"Press any key to continue")

    else:
      stdscr.addstr(6,0,"Fail!")
      stdscr.addstr(8,0,"oni_capture.py log:")

      # Print log from oni_capture
      stdout = iter(p.stdout.readline, b'')
      lastLine = 9
      for line in stdout:
        stdscr.addstr(lastLine,0,line[:-1])
        lastLine = lastLine + 1

      # Delete capture folder
      shutil.rmtree(curOutputDir)

      stdscr.addstr(lastLine+1,0,"Press any key to continue")
      stdscr.refresh()
      quit = True

  # Wait for keypress
  stdscr.getch()

  # Revert terminal to previous settings
  cursesDeinit(stdscr)

  # Print capture summary in original terminal
  print str(numScenesCaptured) + " scenes captured"


################################################################################
# Entry
################################################################################
if __name__ == "__main__":
  main(sys.argv)