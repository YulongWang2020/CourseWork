Part a:
With 5 epochs, the relu activation function gives better accuracy and smaller loss than the sigmoid activation. 
This can be caused by gradient vanishing of the sigmoid activation. The gradient of sigmoid function gets close
 to zero as the training goes. This leads to a slower training rate since the weights and loss changes slower because
 of the small gradient when doing backpropagation. In comparison Relu function have a constant gradient, and
 tanh function has a steeper slope. Both helps resolve this problem.

Part b:
Adam optimizer performs better than SGD optimizer since it adapt the learning rate for each parameter. When
 increasing the momentum, the SGD gives better accuracy because momentum is accelerating the convergence. 
When changing the initializer, the accuracy does not show significant change. 
Using different initializer does not affect the accuracy. This is because the layer is not deep enough so that it is 
easy for the initializer to generate weights that can falls in to global minimum.

Part c:
Using a sigmoid activation, sgd optimizer and random_normal initializer, the gradient is close to zero. 
This is because sigmoid has a small gradient when inputs get larger.
Using a relu activation, adam optimizer and default initializer, the gradient increase from 1E-5 to 1E-2, 
which makes the system converge faster. As mentioned above, relu has a constant gradient helps prevent gradient vanishing.
