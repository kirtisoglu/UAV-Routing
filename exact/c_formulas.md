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

$$
P_0 = \frac{\delta}{8}\rho s A \Omega^3 r^3 
$$

$$
P_i = (1+k) \frac{W^{3/2}}{\sqrt(2\rho A)}
$$

$$
\mu_1 = P_0
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

The energy consumption function is

$$
E(u,d) = \mu_1\frac{d}{v}+\mu_2dv+\mu_3\frac{d}{v^2}+\mu_4dv^2
$$
