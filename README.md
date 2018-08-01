# TC: ThreeCannon

/!\ project discontinued /!\

## The 3 great improvements
 - TC.BodyObject3D is both a body and an object3d, no need to update the mesh position/quaternion at each frame !
 - THREE.Vector3/CANNON.Vec3 and THREE.Quaternion/CANNON.Quaternion are now interchangeable
 - TC.BodyObject3D contain all the information you need in the same place
 
## Is there any cons ?
 - Well currently the API is young and not unit tested so bugs may appear, 
 check issues if you think you encountered one.
 - Rotation property of BodyObject3D is not automatically updated from his quaternion, but you don't use rotation anyway, right ? :D
 
## What does it look like ?
 - You should compare the basic [demo from cannon.js](https://github.com/schteppe/cannon.js/blob/master/examples/threejs.html) 
 with the TC equivalent [demo](https://github.com/gillesdami/TC.js/blob/master/demo/minimal.html), i've added some comments to highlight the differences
 
## Tips to upgrade with TC
 - Your code stay valid if you don't use the property rotation of THREE.Object3d
 - Delete your mechanism to update THREE.object3d instances position from CANNON.Body ones.
 - Replace CANNON.Body by TC.BodyObject3D
 - Add your mesh to the bodyObject and add the bodyObject to what you were adding your mesh
 - Already done, enjoy !
