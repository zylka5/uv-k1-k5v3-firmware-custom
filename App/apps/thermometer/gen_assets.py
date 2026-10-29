#!/usr/bin/env python3
import os, sys
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from app_assets import Assets

# Thermometer nie potrzebuje dużych zasobów - tylko tytuł, wszystko inne rysowane jest bezpośrednio na FB
a = Assets("THERMOMETER")
a.text("T_TITLE", "THERMOMETER")
a.main()
