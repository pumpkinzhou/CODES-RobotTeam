%**************************************************************************
%   File Name     : rot2ZYXeuler.m
%   Author        : Dingjiang Zhou
%                   Boston University, Boston, 02215
%   Create Time   : 
%   Last Modified : 2014/07/16 by Eric Cristofalo
%   Purpose       : rotation matrix to euler angles in ZYX sequence, not
%                   considered the singularity.
%**************************************************************************
function euler = rot2ZYXeuler(R)
% input:
% R, the rotation matrix
% output:
% the euler angles in the ZYX form, as a column vector

euler(1,1) = atan2(R(3,2),R(3,3));      % phi
euler(2,1) = asin(-R(3,1));             % theta
euler(3,1) = atan2(R(2,1),R(1,1));      % psi

end