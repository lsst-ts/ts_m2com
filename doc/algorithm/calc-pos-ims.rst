.. _Calculating_Position_IMS:

##########################################################
Calculating the Position by Independent Measurement System
##########################################################

.. _calc_pos_ims_overview:

Overview
========

This document explains the calculation of rigid body position by using the reading of independent measurement system (IMS).

.. _calc_pos_ims_displacement_sensors:

Displacement Sensors
====================

The following figure shows the location of each pair of displacement sensors, located at the A ring.
There are 4 rings in the M2 mirror: A, B, C, and D.
The 6 tangential actuators are located at A ring with a displacement sensor aside. 
The sensors are A1, A2, A3, A4, A5, and A6 as labeled in the figure.
Every sensor measures two values: :math:`\theta_z` and :math:`\Delta_z`.

.. figure:: ../figure/disp_sensors.png
  :width: 500

  Identification of displacement sensors on M2 cell assembly.

.. _calc_pos_ims_displacement_vectors:

Displacement Vectors
====================

The measurement of displacement sensors gives two vectors: :math:`\vec{d}_{\theta}` and :math:`\vec{d}_{\Delta}`.
Each vector has 6 elements from sensors A1 to A6.
The vector of displacement (:math:`\vec{d}`) is a combination of :math:`\vec{d}_{\theta}` and :math:`\vec{d}_{\Delta}` as the following:

.. math::

    \vec{d} = (d_{\theta, 5}, \ d_{\Delta, 5}, \ d_{\theta, 6}, \ d_{\Delta, 6}, \ d_{\theta, 3}, \ d_{\Delta, 3}, \ d_{\theta, 4}, \ d_{\Delta, 4}, \ d_{\theta, 1}, \ d_{\Delta, 1}, \ d_{\theta, 2}, \ d_{\Delta, 2})

.. _calc_pos_ims_parameters:

Parameters
==========

There are two important elements in the calculation: the offset and the transformation matrix.

.. _calc_pos_ims_offset:

Offset
------

The offset (:math:`\vec{d}_{\text{offset}}`) is the value of displacement sensors recorded when the M2 is at the home position.
This vector might need to be updated if the default home position is changed.
See the value in `disp_ims.yaml <https://github.com/lsst-ts/ts_config_mttcs/blob/develop/MTM2/v2/harrisLUT/disp_ims.yaml>`_.

.. _calc_pos_ims_matrix:

Transformation Matrix
---------------------

The transformation matrix (:math:`\mathbf{M}_{\text{trans}}`) is a 6x12 matrix to transform the readings of IMS to rigid body position :math:`(x, y, z, r_{x}, r_{y}, r_{z})`.
See the matrix in `disp_ims.yaml <https://github.com/lsst-ts/ts_config_mttcs/blob/develop/MTM2/v2/harrisLUT/disp_ims.yaml>`_.

.. _calc_pos_ims_calculation:

Calculation
===========

The following equation calculates the rigid body position:

.. math::

    \begin{pmatrix}
      x \\
      y \\
      z \\
      r_{x} \\
      r_{y} \\
      r_{z}
    \end{pmatrix}
    = \mathbf{M}_{\text{trans}}(\vec{d} - \vec{d}_{\text{offset}})

.. _calc_pos_ims_appendix:

Appendix
========

Define the scaling factors (:math:`s_{r}` and :math:`s_{\theta}`), we have:

.. math::

    \begin{aligned}
    \vec{d} - \vec{d}_{\text{offset}} &=
    \begin{pmatrix}
      s_{r}\cos(60^{\circ}) & -s_{r}\sin(60^{\circ}) & 0 & 0 & 0 & s_{\theta} \\
      0 & 0 & s_{r} & -s_{\theta}\cos(60^{\circ}) & s_{\theta}\sin(60^{\circ}) & 0 \\
      -s_{r}\cos(60^{\circ}) & -s_{r}\sin(60^{\circ}) & 0 & 0 & 0 & s_{\theta} \\
      0 & 0 & s_{r} & s_{\theta}\cos(60^{\circ}) & s_{\theta}\sin(60^{\circ}) & 0 \\
      s_{r}\cos(60^{\circ}) & s_{r}\sin(60^{\circ}) & 0 & 0 & 0 & s_{\theta} \\
      0 & 0 & s_{r} & -s_{\theta}\cos(60^{\circ}) & -s_{\theta}\sin(60^{\circ}) & 0 \\
      s_{r} & 0 & 0 & 0 & 0 & s_{\theta} \\
      0 & 0 & s_{r} & -s_{\theta} & 0 & 0 \\
      -s_{r} & 0 & 0 & 0 & 0 & s_{\theta} \\
      0 & 0 & s_{r} & s_{\theta} & 0 & 0 \\
      -s_{r}\cos(60^{\circ}) & s_{r}\sin(60^{\circ}) & 0 & 0 & 0 & s_{\theta} \\
      0 & 0 & s_{r} & s_{\theta}\cos(60^{\circ}) & -s_{\theta}\sin(60^{\circ}) & 0
    \end{pmatrix}
    \begin{pmatrix}
      x \\
      y \\
      z \\
      r_{x} \\
      r_{y} \\
      r_{z}
    \end{pmatrix} \\
    &= \mathbf{M}
    \begin{pmatrix}
      x \\
      y \\
      z \\
      r_{x} \\
      r_{y} \\
      r_{z}
    \end{pmatrix}
    \end{aligned}

Therefore, the transformation matrix, :math:`\mathbf{M}_{\text{trans}}`, is :math:`\text{pinv}(\mathbf{M})`.
We have :math:`s_{r} = 0.001` mm/um and :math:`s_{\theta} = 1/3600 \times \pi/180 \times R_{M}` mm/arcsec, where :math:`R_{M}=(1781.04^{2} + 9.52^{2})^{1/2}` mm.
The R1 and Z1 positions are used in the calculation of mirror's radius :math:`R_{M}`.
