from building import *
Import('rtconfig')

cwd     = GetCurrentDir()
src	= Glob('*.c')
path = [cwd]

group = DefineGroup('ms5611', src, depend = ['PKG_USING_MS5611'], CPPPATH = path)

Return('group')
