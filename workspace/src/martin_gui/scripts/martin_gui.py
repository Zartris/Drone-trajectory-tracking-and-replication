#!/usr/bin/env python

import sys

from martin_gui.my_module import MyPlugin
from rqt_gui.main import Main

plugin = 'martin_gui'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
