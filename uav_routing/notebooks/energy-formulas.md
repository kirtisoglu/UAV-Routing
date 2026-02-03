| <br /><br /><br />Notation | Description                                    | Reference Values |
| -------------------------- | ---------------------------------------------- | ---------------- |
| $\delta$                 | Profile drag coefficient                       | 0.012            |
| $\rho$                   | Air density$(kg / m^3)$                      | 1.225            |
| $\textit{s}$             | Rotor solidity                                 | 0.05             |
| $\textit{A}$             | Rotor disc area$(m^2)$                       | 0.503            |
| $\Omega$                 | Blade angular velocity$(radian/s)$           | 300              |
| $\textit{r}$             | Rotor radius$(m)$                            | 0.4              |
| $U_{tip}$                | Tip speed of the rotor blade$(m/s)$          | 120              |
| $\textit{k}$             | Incremental correction factor to induced power | 0.1              |
| $v_0$                    | Mean rotor induced velocity in hover$(m/s)$  | 4.03             |
| $d_0$                    | Fuselage drag ratio                            | 0.6              |


The propulsion power function introduced by Zeng et. al. is

$`P(v) \approx P_0 \left( 1 + \frac{3v^2}{U^2_{tip}} \right)$$`

The blade profile power $P_0$, i.e., the power needed to overcome aerodynamic drag of spinning rotor blades is

$$
P_0 = \frac{\delta}{8}\rho s A \Omega^3 r^3.
$$

The induced power $P_i$, which is the power required to generate lift, is

$$
P_i = (1+k) \frac{W^{3/2}}{\sqrt(2\rho A)}
$$

$$
\mu_2 = P_0 d\frac{3}{U^2}
$$

$$
\mu_3 = P_0 v_0
$$

$$
\mu_4 = \frac{1}{2}d_0\rho s A
$$

The propulsion energy required to fly distance $d$ at constant speed $v$ becomes

$$
E(u,d) = \mu_1\frac{d}{v}+\mu_2dv+\mu_3\frac{d}{v^2}+\mu_4dv^2
$$

which has convex monomials for $v >0$ and can be represented using rotated second order cones.


The maximum available energy is calibrated using the same propulsion model employed in the optimization.
Specifically, we define a reference cruise speed $v_{\mathrm{ref}}$ and compute the corresponding propulsion power

$P(v_{\mathrm{ref}})=\mu_1+\mu_2 v_{\mathrm{ref}}^2+\mu_3/v_{\mathrm{ref}}+\mu_4 v_{\mathrm{ref}}^3$

The total energy budget is then set to

$\epsilon_{\max}=P(v_{\mathrm{ref}})\,T^{\mathrm{energy}}_{\max}$, 

which represents the energy required for a continuous flight of duration $T^{\mathrm{energy}}_{\max}$ at cruise speed. This choice
ensures full consistency between the energy budget and the convex energy consumption model used in the formulation.
