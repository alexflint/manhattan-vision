function [ valid ] = occlusion_is_valid( x, left_a, right_a, occl_side, geometry )
%OCCLUSION_VIS_ALID Summary of this function goes here
%   Detailed explanation goes here

%int occl_axis = occl_side < 0 ? left_a : right_a;
if occl_side < 0
    occl_a = left_a;
else
    occl_a = right_a;
end

%int occl_vpt_col = geom->vpt_cols[occl_axis];
occl_vpt_x = geometry.vs(occl_a);

%int occl_vpt_side = occl_vpt_col < col ? -1 : 1;
if occl_vpt_x < x
    occl_vpt_side = -1;
else
    occl_vpt_side = 1;
end

%// irrespective of whether left_axis=right_axis!
%int opp_vpt_col = geom->vpt_cols[1-occl_axis];
opp_vpt_x = geometry.vs(3-occl_a);

%int opp_vpt_side = opp_vpt_col < col ? -1 : 1;
if opp_vpt_x < x
    opp_vpt_side = -1;
else
    opp_vpt_side = 1;
end

%// is the occluding vpt on the same side as the occluding wall?
%bool occl_vpt_behind = occl_side == occl_vpt_side;
occl_vpt_behind = (occl_side == occl_vpt_side);

%// is the opposite vpt between the wall and the occluding vpt?
%bool opp_vpt_between =
%			opp_vpt_side == occl_vpt_side &&
%			abs(col-opp_vpt_col) < abs(col-occl_vpt_col);
opp_vpt_between = (opp_vpt_side == occl_vpt_side) && ...
                  abs(x-opp_vpt_x) < abs(x-occl_vpt_x);

%// the occlusion is valid iff occl_vpt_behind == opp_vpt_between
%return occl_vpt_behind == opp_vpt_between;
valid = (occl_vpt_behind == opp_vpt_between);

end

