# topopt
Topological shape optimization of 2D block

**Description**

This code simulates and renders a constrained system  of 2D points connected together via elastic massless rods (links) forming a 2D block of a solid material.

The links are removed/deactivated from the system, once their change in length is less than a predefined small number.
Thus, the block losses its mass in those regions, where the construction have low values of elastic strain.

The links or rods are rendered and color-coded w.r.t. their value of strain. 
The colormap is computed on the go as a weighted sum of 3 slightly overlapping Gaussians that contribute to 3 principal color values: blue, green and red.

The physics simulation is based on time-explicit integrating (via the Verlet's formula) of momentum equations for each particle and resolving their connections moving the particles in opposite directions along the link.

To run simply open up the [index.html](index.html) in any browser.

**Demo**

![Demo](./4readme/topopt.gif)

**License**

Published under the [MIT License](LICENSE).
