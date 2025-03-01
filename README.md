
# Line Disc Distance

I recently had to determine the maximal distance between a line and a disc, and the solution I came up with is shared here. Let's imagine a cylinder whose axis is the line l. The distance we are looking for is the radius of that cylinder so that it is tangent to, and contains, the disc.

![Animation](Animation.gif)

We consider the plane the disc lies on. That plane cuts the cylinder, their intersection is an ellipse. We know the "aspect" (s) (ratio of the minor and major axes) of that ellipse, for it only depends on the angle of incidence of the line with respect to the plane.

We now want to evaluate the values of the minor and major semi axes of the ellipse. The ellipse is tangent to the disc it contains. We therefore look for a point lying both on the ellipse and the circle, so that the normal to the ellipse at that point passes through the disc center.

We evaluate this point and the ellipse iteratively. We consider the ellipse of aspect s passing through the disc center. We are looking for an ellipse of identical aspect, scaled up so that it is tangent to the disc.

* (1) We evaluate the normal to the current ellipse at the disc center.<br />
* (2) We consider the point lying on that normal at distance r (the radius of the disc).<br />
* (3) We evaluate the ellipse passing through that point.<br />
* (4) We evaluate the normal to that ellipse at that point.<br />
* (5) We repeat the procedure using that normal, from step (2).<br />
