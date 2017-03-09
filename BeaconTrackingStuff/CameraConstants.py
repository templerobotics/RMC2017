#Calibrated using 600 images each, March 3 2017, Sam Wilson
import numpy as np

bfx = 379.7279
bfy = 380.5608
bcx = 292.7610
bcy = 226.6971
br1 = -0.3290
br2 = 0.1545
br3 = -0.0413
billMatrix = np.matrix([ [bfx, 0, bcx], [0, bfy, bcy], [0, 0, 1] ])
billDistort = np.matrix([ br1, br2, 0, 0, br3])

tfx = 373.7896
tfy = 374.5872
tcx = 317.5320
tcy = 237.4582
tr1 = -0.3321
tr2 = 0.1574
tr3 = -0.04267
tedMatrix = np.matrix([ [tfx, 0, tcx], [0, tfy, tcy], [0, 0, 1] ])
tedDistort = np.matrix([ tr1, tr2, 0, 0, tr3])
