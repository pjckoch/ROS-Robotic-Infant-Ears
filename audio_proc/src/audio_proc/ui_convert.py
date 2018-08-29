#!/usr/bin/env python

"""
The following is a summary of the licenses involved in this project.
Please also refer to the LICENSE folder in this github repository
for full licensing information.

LICENSE SUMMARY:
------------------------------------------
           The MIT License (MIT)

applies to:
- this file, Copyright (c) 2016 Scott W Harden
------------------------------------------
              GNU GPL License

applies to:
- PyQt4, Copyright (C) 2011 Riverbank Computing Limited
         Note: Redistribution possible under compatible licenses
               (see https://www.gnu.org/licenses/license-list.en.html)
------------------------------------------
"""

from PyQt4 import uic
import glob
for fname in glob.glob("*.ui"):
    print("converting",fname)
    fin = open(fname,'r')
    fout = open(fname.replace(".ui",".py"),'w')
    uic.compileUi(fin,fout,execute=False)
    fin.close()
    fout.close()
