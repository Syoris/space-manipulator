function Jac = geometricJacobian(obj, Q, endeffectorname)
%geometricJacobian Compute the geometric Jacobian
%   JAC = geometricJacobian(ROBOT, Q, ENDEFFECTORNAME) computes
%   the geometric Jacobian for the body ENDEFFECTORNAME in ROBOT
%   under the configuration Q. The Jacobian matrix JAC is of size
%   6xN, where N is the number of degrees of freedom. The
%   Jacobian maps joint-space velocity to the Cartesian space
%   end-effector velocity relative to the base coordinate frame.
%
%   Example:
%       % Load predefined robot models
%       load exampleRobots
%
%       % Get the Jacobian for right_wrist body in Baxter robot
%       % under a random configuration
%       jac = geometricJacobian(baxter,...
%                      baxter.randomConfiguration,'right_wrist');

    Jac = obj.TreeInternal.geometricJacobian(Q, endeffectorname);
    warning('Not yep implemented');
end