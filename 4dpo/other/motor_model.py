import numpy as np
from scipy import optimize
import matplotlib.pyplot as plt



def main():
  #plt.style.use('seaborn-poster')
  # generate x and y
  x = np.linspace(0, 1, 101)
  y = 1 + x + x * np.random.random(len(x))
  # assemble matrix A
  A = np.vstack([x, np.ones(len(x))]).T

  # turn y into a column vector
  y = y[:, np.newaxis]

  # Direct least square regression
  alpha = np.dot((np.dot(np.linalg.inv(np.dot(A.T,A)),A.T)),y)
  print(alpha)

  # plot the results
  plt.figure(figsize = (10,8))
  plt.plot(x, y, 'b.')
  plt.plot(x, alpha[0]*x + alpha[1], 'r')
  plt.xlabel('x')
  plt.ylabel('y')
  plt.show()

if __name__ == "__main__":
    main()