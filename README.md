# IDRCS
Interval-Driven Robust Control Suite

This suite is developed to estimate the negative-definite and invariant set in the state-control space of a specific plant (plant nominal model see allinone***.txt). Run [robust_asy_IA_combine](https://github.com/CharlieLuuke/IDRCS/blob/master/robust_asy_IA_combine.py) to get the negative-definite domain. Run [robust_ia_map](https://github.com/CharlieLuuke/IDRCS/blob/master/robust_ia_map.py) to get the negative-definite and invariant set, the result will be saved as *.csv.

[pyIbex](http://benensta.github.io/pyIbex/) is needed.

![Image text](https://github.com/CharlieLuuke/IDRCS/blob/master/Sample.bmp)

RNS-SC Computation time:
|Timing(s)\Platform | i5 3.2 GHZ |Athlon 2.7 GHZ   |
| :---- | ----: | :----: | 
|$L(x)=x^2$  | 21.16 | 43.69   |
|$L^\ast(x)$ | 187.23 | 346.72  |
