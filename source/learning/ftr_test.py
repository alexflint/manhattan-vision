import math
import matplotlib.pyplot as plt

import py_indoor_context

mgr = py_indoor_context.TrainingManager()
mgr.LoadSequence('lab_kitchen1', [5])
mgr.ComputeMonoFeatures("all,-nbr_sweeps", True)

inst = mgr.GetInstance(0)


fgen = py_indoor_context.PhotometricFeatures()
fgen.Configure("all,-nbr_sweeps")
fgen.Compute(inst)

for i in range(fgen.NumFeatures()):
    print i,':',fgen.GetFeatureName(i)

print 'Payoffs Features:'
for i in range(inst.NumFeatures()):
    print i,':',inst.GetFeatureComment(i)

ftr_ids = [1,24,27,28,29]
po_ids = [72,73,74]

print '\nDISPLAYING:'

n = len(ftr_ids) + len(po_ids)
nx = int(math.ceil(math.sqrt(n)))
ny = int(math.ceil(float(n) / nx))

print nx,ny

plt.clf()
for ii,i in enumerate(ftr_ids):
    print fgen.GetFeatureName(i)
    plt.subplot(ny, nx ,ii+1)
    plt.imshow(fgen.GetFeature(i))
    plt.title(fgen.GetFeatureName(i))

for ii,i in enumerate(po_ids):
    print inst.GetFeatureComment(i)
    plt.subplot(ny, nx ,len(ftr_ids)+ii+1)
    plt.imshow(inst.GetFeature(i,0))
    plt.title(inst.GetFeatureComment(i))
    
plt.savefig('features.pdf')
