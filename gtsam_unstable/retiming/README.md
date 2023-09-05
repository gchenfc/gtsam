# Trajectory Retiming

This folder contains code for solving trajectory retiming problems using variable elimination.


## "Scalar" LP/QP Solver
The core algorithm for solving the trajectory retiming problem is essentially an LP solver (extend to QP in [Chen24icra](^1)) for the special case when the separator for any elimination step involving inequality constraints is exactly 1 scalar variable.  This special case permits efficiently solving the inequality constraints using standard variable elimination (in linear time) because the resulting joint distribution ("new factor") on the separator will always be on exactly 1 variable, so the inequality constraints can always be reduced to at most 2 constraints (lower and upper bound).  In contrast, the general case of attempting variable elimination on a graph with inequality constraints will result in the number of inequality constraints potentially growing exponentially (see Fourier-Motzkin Elimination).

One hiccup in this approach is that the objective component ("cost-to-go") will still grow linearly with the number of constraints in the worst case, although (1) for the time-optimal retiming problem, we can take a greedy choice of each variable so we can totally ignore the objective function, and (2) we will almost never actually see this worst-case behavior in practice.

## Time-Optimal Path Parameterization Formulation

I recommend reviewing [Pham18tro](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=8338417), since for someone familiar with factor graphs and variable elimination, it will jump out at you that their SoTA algorithm based on "reachability analysis" is very clearly variable elimination.  Other texts before this paper tend to be moored in bang-bang terminology with switching points which is farther from the variable elimination mentality, but some texts you may refer to include Ch. 11 of Choset et al's Principles of Robot Motion, or Ch. 9.4 of Lynch & Park's Modern Robotics.

Define our retiming as $s(t): [0, T] \rightarrow [0, 1]$ is (strictly) monotonically increasing.
After the expected substitutions into the EOM and constraints, we will arrive at a scalar nonlinear optimization problem of the form:

$$
\begin{align*}
\min_s ~&T\\
\mathrm{s.t.} ~~&a(s)\ddot{s} + b(s)\dot{s}^2 + c(s) \in \mathcal{C}(s), &\forall t\\
\end{align*}
$$

where $a(s), b(s), c(s)$ are vector-valued -- i.e. there can be many constraints -- and $\mathcal{C}$ is a convex polytope.  Note that $a(s)$ can equal 0 to enable first-order constraints.  In the bang-bang approaches, special considerations need to be made for when these constraints become active ("zero-inertia" points), but in the variable elimination approach we can just treat them as any other constraint.

As expected, we discretize the problem (in $s$):

$$
\begin{align*}
\min_{s_0, \ldots, s_N} ~&T\\
\mathrm{s.t.} ~~&a_k\ddot{s}_k + b_k\dot{s}_k^2 + c_k \in \mathcal{C}_k, &\forall t\\
\end{align*}
$$

Now, the key trick that the TOPP literature discovered decades ago is that we now reparameterize this problem as
$$x_k:=\ddot{s}_k$$
$$u_k:=\dot{s}_k^2$$
And because $\frac{dx}{ds} = \frac{d}{ds} \dot{s}^2 = 2\dot{s}\frac{d\dot{s}}{ds} = 2\frac{d\dot{s}}{ds}\dot{s} = 2\ddot{s} = 2u$, then we introduce the new "dynamics" constraint
$$ x_{k+1} = x_k + 2u_k\Delta s, \quad k=0,\ldots, N-1$$

Then finally, putting everything together, we have this optimization problem with convex constraints:

$$
\begin{align*}
\min_{x_0, u_0, \ldots, x_N} ~&T\\
\mathrm{s.t.} ~~&a_ku + b_kx + c_k \in \mathcal{C}_k, &&k=0,\ldots, N\\
  &x_{k+1} = x_k + 2u_k\Delta s, &&k=0,\ldots,N-1\\
\end{align*}
$$

## Factor Graph Approach
We will define the following factor types:
* Objective - normal factors.  TODO: "greedy" vs "quadratic" objectives.  These are eliminated as usual.
* Linear Equality Constraints - these are eliminated as usual (substitution).  Since we have only scalar variables, we don't have to worry about null space projection etc. that would be handled by Gauss-Jordan elimination in GaussianFactorGraph elimination.
* Linear Inequality Constraints - these are eliminated using a custom thing.

The result of eliminating a variable will be:
1. 3 new factors on the separator: one for each constraint type
2. 1 conditional distribution which is either an delayed-solve LP (greedy objective) or a piecewise quadratic (quadratic objective).

Of course, to follow the GTSAM traits/templating, the returned separator will actually need to be a single factor, so we will actually make every factor able to hold any combination of the 3 "factor types", and the conditional distribution will be a polymorphic type for the "greedy" (delayed LP solve) vs "quadratic" (piecewise quadratic) types.

## Footnotes
[^1]: TODO: fill-in this with arXiv and/or icra link if accepted
