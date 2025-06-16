import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sofia/sextoSem/robotica_inteligente/humanoides/disney/install/marker'
