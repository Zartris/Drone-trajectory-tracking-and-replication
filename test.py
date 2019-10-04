import numpy as np
     
#z = [0.0, 0.695, 1.345, 1.865, 2.225, 2.590, 0.0, 0.719, 1.405, 1.978, 2.398, 2.730, 0.0, 0.789, 1.474, 2.064, 2.472, 2.775, 0.0, 0.763, 1.453, 1.968, 2.356, 2.649]

z= [0.0, 0.695, 1.345]
     
#x = [0.0, 12.0, 24.0, 36.0, 48.0, 60.0, 0.0, 12.0, 24.0, 36.0, 48.0, 60.0, 0.0, 12.0, 24.0, 36.0, 48.0, 60.0, 0.0, 12.0, 24.0, 36.0, 48.0, 60.0]
x = [0.0, 12.0, 24.0]
#y = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 24.0, 24.0, 24.0, 24.0, 24.0, 24.0, 48.0, 48.0, 48.0, 48.0, 48.0, 48.0, 72.0, 72.0, 72.0, 72.0, 72.0, 72.0]
y = [2, 1, 3]
degree = 3
thickness = 0.167

# Set up the canonical least squares form
Ax = np.vander(x, degree)
Ay = np.vander(y, degree)
A = np.hstack((Ax, Ay))

# Solve for a least squares estimate
(coeffs, residuals, rank, sing_vals) = np.linalg.lstsq(A, z)

# Extract coefficients and create polynomials in x and y
xcoeffs = coeffs[0:degree]
ycoeffs = coeffs[degree:2 * degree]

fx = np.poly1d(xcoeffs)
fy = np.poly1d(ycoeffs)

print(fx)
print(fy)


t = np.arange(len(y))  # simple assumption that data was sampled in regular steps
print( 0, 2,0)
print( 12.0, 1,0.695)
print( 24.0, 3,1.345)

for d in range(3):
	fitx = np.polyfit(t, x, d)
	fity = np.polyfit(t, y, d)
	fitz = np.polyfit(t, z, d)
	fx = np.poly1d(fitx)
	fy = np.poly1d(fity)
	fz = np.poly1d(fitz)
	print(0, d, fx(0), fy(0),fz(0))
	print(1, d, fx(1), fy(1),fz(1))
	print(2, d, fx(2), fy(2),fz(2))

print(fx(0.5), fy(0.5),fz(0.5))
	
