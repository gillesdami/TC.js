window.TC = window.TC || {};

(function(TC) {
    let id = Math.pow(2,32);//to not collide with Object3d/Cannon.Body objects id counters

    //VECTOR
    var antip_neg = new CANNON.Vec3();
    var Vec3_tangents_n = new CANNON.Vec3();
    var Vec3_tangents_randVec = new CANNON.Vec3();
    CANNON.Vec3.prototype.tangents = function(t1,t2){
        var norm = this.norm();
        if(norm>0.0){
            var n = Vec3_tangents_n;
            var inorm = 1/norm;
            n.set(this.x*inorm,this.y*inorm,this.z*inorm);
            var randVec = Vec3_tangents_randVec;
            if(Math.abs(n.x) < 0.9){
                randVec.set(1,0,0);
                n.cross(randVec,t1);
            } else {
                randVec.set(0,1,0);
                n.cross(randVec,t1);
            }
            n.cross(t1,t2);
        } else {
            // The normal length is zero, make something up
            t1.set(1, 0, 0);
            t2.set(0, 1, 0);
        }
    };
    CANNON.Vec3.prototype.cross = function(v,target){
        var x = this.x, y = this.y, z = this.z;

        if(target) {
            var vx=v.x, vy=v.y, vz=v.z;
            target = target || new CANNON.Vec3();

            target.x = (y * vz) - (z * vy);
            target.y = (z * vx) - (x * vz);
            target.z = (x * vy) - (y * vx);

            return target;
        }
        else {
            this.x = y * v.z - z * v.y;
            this.y = z * v.x - x * v.z;
            this.z = x * v.y - y * v.x;

            return this;
        }
    };
    CANNON.Vec3.prototype.negate = function(target){
        if(target) {
            target = target || new CANNON.Vec3();
            target.x = -this.x;
            target.y = -this.y;
            target.z = -this.z;
            return target;
        }
        else {
            this.x = - this.x;
            this.y = - this.y;
            this.z = - this.z;

            return this;
        }
    };
    CANNON.Vec3.prototype.isAntiparallelTo = function(v,precision){
        this.negate(antip_neg);
        return antip_neg.almostEquals(v,precision);
    };
    CANNON.Vec3.prototype.lerp = function ( v, alpha, target ) {

        if(target) {
            var t = alpha;
            var x=this.x, y=this.y, z=this.z;
            target.x = x + (v.x-x)*t;
            target.y = y + (v.y-y)*t;
            target.z = z + (v.z-z)*t;
        }
        else {
            this.x += ( v.x - this.x ) * alpha;
            this.y += ( v.y - this.y ) * alpha;
            this.z += ( v.z - this.z ) * alpha;

            return this;
        }
    };
    CANNON.Vec3.prototype.toArray = function ( array, offset ) {

        if ( array === undefined ) array = [];
        if ( offset === undefined ) offset = 0;

        array[ offset ] = this.x;
        array[ offset + 1 ] = this.y;
        array[ offset + 2 ] = this.z;

        return array;

    };
    Object.assign(THREE.Vector3.prototype, CANNON.Vec3.prototype);
    Object.assign(CANNON.Vec3.prototype, THREE.Vector3.prototype);
    Object.assign(THREE.Vector3, CANNON.Vec3);
    Object.assign(CANNON.Vec3, THREE.Vector3);

    //QUATERNION
    var sfv_t1 = new CANNON.Vec3(),
        sfv_t2 = new CANNON.Vec3();
    var Quaternion_mult_va = new CANNON.Vec3();
    var Quaternion_mult_vb = new CANNON.Vec3();
    var Quaternion_mult_vaxvb = new CANNON.Vec3();
    Object.defineProperties(CANNON.Quaternion.prototype, {
        onChangeCallback: {
            get: function () {
                return () => {};
            },
            set: function ( value ) {
            }
        }
    } );
    Object.defineProperties(CANNON.Quaternion.prototype, {

        x: {

            get: function () {
                return this._x;
            },

            set: function ( value ) {
                this._x = value;
            }

        },
        y: {
            get: function () {
                return this._y;
            },
            set: function ( value ) {
                this._y = value;
            }
        },

        z: {
            get: function () {
                return this._z;
            },

            set: function ( value ) {

                this._z = value;

            }

        },

        w: {

            get: function () {

                return this._w;

            },

            set: function ( value ) {

                this._w = value;

            }

        }

    } );
    Object.assign( CANNON.Quaternion.prototype, {

        set: function ( x, y, z, w ) {

            this.x = x;
            this.y = y;
            this.z = z;
            this.w = w;

            return this;

        }
    } );
    CANNON.Quaternion.prototype.mult = function(q,target){
        target = target || new CANNON.Quaternion();
        var w = this.w,
            va = Quaternion_mult_va,
            vb = Quaternion_mult_vb,
            vaxvb = Quaternion_mult_vaxvb;

        va.set(this.x,this.y,this.z);
        vb.set(q.x,q.y,q.z);
        target.w = w*q.w - va.dot(vb);
        va.cross(vb,vaxvb);

        target.x = w * vb.x + q.w*va.x + vaxvb.x;
        target.y = w * vb.y + q.w*va.y + vaxvb.y;
        target.z = w * vb.z + q.w*va.z + vaxvb.z;

        return target;
    };
    CANNON.Quaternion.prototype.setFromEuler = function ( euler, update, z, order ) {

        if ( ! ( euler && euler.isEuler ) ) {

            euler = new THREE.Euler(euler, update, z, order);
            update = false;
        }

        var x = euler._x, y = euler._y, z = euler._z, order = euler.order;

        // http://www.mathworks.com/matlabcentral/fileexchange/
        // 	20696-function-to-convert-between-dcm-euler-angles-quaternions-and-euler-vectors/
        //	content/SpinCalc.m

        var cos = Math.cos;
        var sin = Math.sin;

        var c1 = cos( x / 2 );
        var c2 = cos( y / 2 );
        var c3 = cos( z / 2 );

        var s1 = sin( x / 2 );
        var s2 = sin( y / 2 );
        var s3 = sin( z / 2 );

        if ( order === 'XYZ' ) {

            this._x = s1 * c2 * c3 + c1 * s2 * s3;
            this._y = c1 * s2 * c3 - s1 * c2 * s3;
            this._z = c1 * c2 * s3 + s1 * s2 * c3;
            this._w = c1 * c2 * c3 - s1 * s2 * s3;

        } else if ( order === 'YXZ' ) {

            this._x = s1 * c2 * c3 + c1 * s2 * s3;
            this._y = c1 * s2 * c3 - s1 * c2 * s3;
            this._z = c1 * c2 * s3 - s1 * s2 * c3;
            this._w = c1 * c2 * c3 + s1 * s2 * s3;

        } else if ( order === 'ZXY' ) {

            this._x = s1 * c2 * c3 - c1 * s2 * s3;
            this._y = c1 * s2 * c3 + s1 * c2 * s3;
            this._z = c1 * c2 * s3 + s1 * s2 * c3;
            this._w = c1 * c2 * c3 - s1 * s2 * s3;

        } else if ( order === 'ZYX' ) {

            this._x = s1 * c2 * c3 - c1 * s2 * s3;
            this._y = c1 * s2 * c3 + s1 * c2 * s3;
            this._z = c1 * c2 * s3 - s1 * s2 * c3;
            this._w = c1 * c2 * c3 + s1 * s2 * s3;

        } else if ( order === 'YZX' ) {

            this._x = s1 * c2 * c3 + c1 * s2 * s3;
            this._y = c1 * s2 * c3 + s1 * c2 * s3;
            this._z = c1 * c2 * s3 - s1 * s2 * c3;
            this._w = c1 * c2 * c3 - s1 * s2 * s3;

        } else if ( order === 'XZY' ) {

            this._x = s1 * c2 * c3 - c1 * s2 * s3;
            this._y = c1 * s2 * c3 - s1 * c2 * s3;
            this._z = c1 * c2 * s3 + s1 * s2 * c3;
            this._w = c1 * c2 * c3 + s1 * s2 * s3;

        }

        if ( update !== false ) this.onChangeCallback();

        return this;

    };
    CANNON.Quaternion.prototype.inverse = function (target) {
        if(target) {
            var x = this.x, y = this.y, z = this.z, w = this.w;
            target = target || new CANNON.Quaternion();

            this.conjugate(target);
            var inorm2 = 1/(x*x + y*y + z*z + w*w);
            target.x *= inorm2;
            target.y *= inorm2;
            target.z *= inorm2;
            target.w *= inorm2;

            return target;
        }
        else {
            return this.conjugate().normalize();
        }

    };
    CANNON.Quaternion.prototype.conjugate = function (target) {
        if(target) {
            target = target || new CANNON.Quaternion();

            target.x = -this.x;
            target.y = -this.y;
            target.z = -this.z;
            target.w = this.w;

            return target;
        }
        else {
            this._x *= - 1;
            this._y *= - 1;
            this._z *= - 1;

            this.onChangeCallback();

            return this;
        }
    };
    CANNON.Quaternion.prototype.setFromVectors = function(u,v){
        if(u.isAntiparallelTo(v)){
            var t1 = sfv_t1;
            var t2 = sfv_t2;

            u.tangents(t1,t2);
            this.setFromAxisAngle(t1,Math.PI);
        } else {
            var a = u.cross(v);
            this.x = a.x;
            this.y = a.y;
            this.z = a.z;
            this.w = Math.sqrt(Math.pow(u.norm(),2) * Math.pow(v.norm(),2)) + u.dot(v);
            this.normalize();
        }
    };

    Object.assign(THREE.Quaternion.prototype, CANNON.Quaternion.prototype);
    Object.assign(CANNON.Quaternion.prototype, THREE.Quaternion.prototype);
    Object.assign(CANNON.Quaternion, THREE.Quaternion);
    Object.assign(THREE.Quaternion, CANNON.Quaternion);

    //BODY_OBJECT
    /**
     * use three.js/src/core/EventDispatcher.js over cannon.js/src/utils/EventTarget
     *
     * THREE.Object3D and CANNON.Body mixin
     */
    TC.BodyObject3D = class BodyObject3D {
        constructor(options) {
            //------THREE------//

            this.id = id++;
            this.uuid = THREE.Math.generateUUID();

            this.name = '';
            this.type = 'Object3D';

            this.parent = null;
            this.children = [];

            this.up = TC.BodyObject3D.DefaultUp.clone();

            var position = new THREE.Vector3();
            var rotation = new THREE.Euler();
            var quaternion = new THREE.Quaternion();
            var scale = new THREE.Vector3( 1, 1, 1 );

            function onRotationChange() {

                quaternion.setFromEuler( rotation, false );

            }

            function onQuaternionChange() {

                rotation.setFromQuaternion( quaternion, undefined, false );

            }

            rotation.onChange( onRotationChange );
            quaternion.onChange( onQuaternionChange );


            Object.defineProperties( this, {
                position: {
                    enumerable: true,
                    value: position
                },
                rotation: {
                    enumerable: true,
                    value: rotation
                },
                quaternion: {
                    enumerable: true,
                    value: quaternion
                },
                scale: {
                    enumerable: true,
                    value: scale
                },
                modelViewMatrix: {
                    value: new THREE.Matrix4()
                },
                normalMatrix: {
                    value: new THREE.Matrix3()
                }
            } );

            this.matrix = new THREE.Matrix4();
            this.matrixWorld = new THREE.Matrix4();

            this.matrixAutoUpdate = TC.BodyObject3D.DefaultMatrixAutoUpdate;
            this.matrixWorldNeedsUpdate = false;

            this.layers = new THREE.Layers();
            this.visible = true;

            this.castShadow = false;
            this.receiveShadow = false;

            this.frustumCulled = true;
            this.renderOrder = 0;

            this.userData = {};

            //------CANNON------//
            if(!options) options = {};

            /**
             * Reference to the world the body is living in
             * @property world
             * @type {World}
             */
            this.world = null;

            /**
             * Callback function that is used BEFORE stepping the system. Use it to apply forces, for example. Inside the function, "this" will refer to this Body object.
             * @property preStep
             * @type {Function}
             * @deprecated Use World events instead
             */
            this.preStep = null;

            /**
             * Callback function that is used AFTER stepping the system. Inside the function, "this" will refer to this Body object.
             * @property postStep
             * @type {Function}
             * @deprecated Use World events instead
             */
            this.postStep = null;

            this.vlambda = new CANNON.Vec3();

            /**
             * @property {Number} collisionFilterGroup
             */
            this.collisionFilterGroup = typeof(options.collisionFilterGroup) === 'number' ? options.collisionFilterGroup : 1;

            /**
             * @property {Number} collisionFilterMask
             */
            this.collisionFilterMask = typeof(options.collisionFilterMask) === 'number' ? options.collisionFilterMask : -1;

            /**
             * Whether to produce contact forces when in contact with other bodies. Note that contacts will be generated, but they will be disabled.
             * @property {Number} collisionResponse
             */
            this.collisionResponse = true;

            /**
             * World space position of the body.
             * @property position
             * @type {Vec3}
             */
            //this.position = new CANNON.Vec3(); !done by three

            /**
             * @property {Vec3} previousPosition
             */
            this.previousPosition = new CANNON.Vec3();

            /**
             * Interpolated position of the body.
             * @property {Vec3} interpolatedPosition
             */
            this.interpolatedPosition = new CANNON.Vec3();

            /**
             * Initial position of the body
             * @property initPosition
             * @type {Vec3}
             */
            this.initPosition = new CANNON.Vec3();
            if(options.position){
                this.position.copy(options.position);
                this.previousPosition.copy(options.position);
                this.interpolatedPosition.copy(options.position);
                this.initPosition.copy(options.position);
            }

            /**
             * World space velocity of the body.
             * @property velocity
             * @type {Vec3}
             */
            this.velocity = new CANNON.Vec3();

            if(options.velocity){
                this.velocity.copy(options.velocity);
            }

            /**
             * @property initVelocity
             * @type {Vec3}
             */
            this.initVelocity = new CANNON.Vec3();

            /**
             * Linear force on the body in world space.
             * @property force
             * @type {Vec3}
             */
            this.force = new CANNON.Vec3();

            var mass = typeof(options.mass) === 'number' ? options.mass : 0;

            /**
             * @property mass
             * @type {Number}
             * @default 0
             */
            this.mass = mass;

            /**
             * @property invMass
             * @type {Number}
             */
            this.invMass = mass > 0 ? 1.0 / mass : 0;

            /**
             * @property material
             * @type {Material}
             */
            this.material = options.material || null;

            /**
             * @property linearDamping
             * @type {Number}
             */
            this.linearDamping = typeof(options.linearDamping) === 'number' ? options.linearDamping : 0.01;

            /**
             * One of: BodyObject3D.DYNAMIC, BodyObject3D.STATIC and BodyObject3D.KINEMATIC.
             * @property type
             * @type {Number}
             */
            this.type = (mass <= 0.0 ? BodyObject3D.STATIC : BodyObject3D.DYNAMIC);
            if(typeof(options.type) === typeof(BodyObject3D.STATIC)){
                this.type = options.type;
            }

            /**
             * If true, the body will automatically fall to sleep.
             * @property allowSleep
             * @type {Boolean}
             * @default true
             */
            this.allowSleep = typeof(options.allowSleep) !== 'undefined' ? options.allowSleep : true;

            /**
             * Current sleep state.
             * @property sleepState
             * @type {Number}
             */
            this.sleepState = 0;

            /**
             * If the speed (the norm of the velocity) is smaller than this value, the body is considered sleepy.
             * @property sleepSpeedLimit
             * @type {Number}
             * @default 0.1
             */
            this.sleepSpeedLimit = typeof(options.sleepSpeedLimit) !== 'undefined' ? options.sleepSpeedLimit : 0.1;

            /**
             * If the body has been sleepy for this sleepTimeLimit seconds, it is considered sleeping.
             * @property sleepTimeLimit
             * @type {Number}
             * @default 1
             */
            this.sleepTimeLimit = typeof(options.sleepTimeLimit) !== 'undefined' ? options.sleepTimeLimit : 1;

            this.timeLastSleepy = 0;

            this._wakeUpAfterNarrowphase = false;

            /**
             * World space rotational force on the body, around center of mass.
             * @property {Vec3} torque
             */
            this.torque = new CANNON.Vec3();

            /**
             * World space orientation of the body.
             * @property quaternion
             * @type {Quaternion}
             */
            //this.quaternion = new THREE.Quaternion(); !done by three

            /**
             * @property initQuaternion
             * @type {Quaternion}
             */
            this.initQuaternion = new THREE.Quaternion();

            /**
             * @property {Quaternion} previousQuaternion
             */
            this.previousQuaternion = new THREE.Quaternion();

            /**
             * Interpolated orientation of the body.
             * @property {Quaternion} interpolatedQuaternion
             */
            this.interpolatedQuaternion = new THREE.Quaternion();

            if(options.quaternion){
                this.quaternion.copy(options.quaternion);
                this.initQuaternion.copy(options.quaternion);
                this.previousQuaternion.copy(options.quaternion);
                this.interpolatedQuaternion.copy(options.quaternion);
            }

            /**
             * Angular velocity of the body, in world space. Think of the angular velocity as a vector, which the body rotates around. The length of this vector determines how fast (in radians per second) the body rotates.
             * @property angularVelocity
             * @type {Vec3}
             */
            this.angularVelocity = new CANNON.Vec3();

            if(options.angularVelocity){
                this.angularVelocity.copy(options.angularVelocity);
            }

            /**
             * @property initAngularVelocity
             * @type {Vec3}
             */
            this.initAngularVelocity = new CANNON.Vec3();

            /**
             * @property shapes
             * @type {array}
             */
            this.shapes = [];

            /**
             * Position of each Shape in the body, given in local Body space.
             * @property shapeOffsets
             * @type {array}
             */
            this.shapeOffsets = [];

            /**
             * Orientation of each Shape, given in local Body space.
             * @property shapeOrientations
             * @type {array}
             */
            this.shapeOrientations = [];

            /**
             * @property inertia
             * @type {Vec3}
             */
            this.inertia = new CANNON.Vec3();

            /**
             * @property {Vec3} invInertia
             */
            this.invInertia = new CANNON.Vec3();

            /**
             * @property {Mat3} invInertiaWorld
             */
            this.invInertiaWorld = new CANNON.Mat3();

            this.invMassSolve = 0;

            /**
             * @property {Vec3} invInertiaSolve
             */
            this.invInertiaSolve = new CANNON.Vec3();

            /**
             * @property {Mat3} invInertiaWorldSolve
             */
            this.invInertiaWorldSolve = new CANNON.Mat3();

            /**
             * Set to true if you don't want the body to rotate. Make sure to run .updateMassProperties() after changing this.
             * @property {Boolean} fixedRotation
             * @default false
             */
            this.fixedRotation = typeof(options.fixedRotation) !== "undefined" ? options.fixedRotation : false;

            /**
             * @property {Number} angularDamping
             */
            this.angularDamping = typeof(options.angularDamping) !== 'undefined' ? options.angularDamping : 0.01;

            /**
             * Use this property to limit the motion along any world axis. (1,1,1) will allow motion along all axes while (0,0,0) allows none.
             * @property {Vec3} linearFactor
             */
            this.linearFactor = new CANNON.Vec3(1,1,1);
            if(options.linearFactor){
                this.linearFactor.copy(options.linearFactor);
            }

            /**
             * Use this property to limit the rotational motion along any world axis. (1,1,1) will allow rotation along all axes while (0,0,0) allows none.
             * @property {Vec3} angularFactor
             */
            this.angularFactor = new CANNON.Vec3(1,1,1);
            if(options.angularFactor){
                this.angularFactor.copy(options.angularFactor);
            }

            /**
             * World space bounding box of the body and its shapes.
             * @property aabb
             * @type {AABB}
             */
            this.aabb = new CANNON.AABB();

            /**
             * Indicates if the AABB needs to be updated before use.
             * @property aabbNeedsUpdate
             * @type {Boolean}
             */
            this.aabbNeedsUpdate = true;

            /**
             * Total bounding radius of the Body including its shapes, relative to body.position.
             * @property boundingRadius
             * @type {Number}
             */
            this.boundingRadius = 0;

            this.wlambda = new CANNON.Vec3();

            if(options.shape){
                this.addShape(options.shape);
            }

            this.updateMassProperties();
        }
    };

    //------THREE------//

    TC.BodyObject3D.DefaultUp = new THREE.Vector3( 0, 1, 0 );
    TC.BodyObject3D.DefaultMatrixAutoUpdate = true;

    Object.assign( TC.BodyObject3D.prototype, THREE.EventDispatcher.prototype, {

        isObject3D: true,

        onBeforeRender: function () {},
        onAfterRender: function () {},

        applyMatrix: function ( matrix ) {

            this.matrix.multiplyMatrices( matrix, this.matrix );

            this.matrix.decompose( this.position, this.quaternion, this.scale );

        },

        applyQuaternion: function ( q ) {

            this.quaternion.premultiply( q );

            return this;

        },

        setRotationFromAxisAngle: function ( axis, angle ) {

            // assumes axis is normalized

            this.quaternion.setFromAxisAngle( axis, angle );

        },

        setRotationFromEuler: function ( euler ) {

            this.quaternion.setFromEuler( euler, true );

        },

        setRotationFromMatrix: function ( m ) {

            // assumes the upper 3x3 of m is a pure rotation matrix (i.e, unscaled)

            this.quaternion.setFromRotationMatrix( m );

        },

        setRotationFromQuaternion: function ( q ) {

            // assumes q is normalized

            this.quaternion.copy( q );

        },

        rotateOnAxis: function () {

            // rotate object on axis in object space
            // axis is assumed to be normalized

            var q1 = new THREE.Quaternion();

            return function rotateOnAxis( axis, angle ) {

                q1.setFromAxisAngle( axis, angle );

                this.quaternion.multiply( q1 );

                return this;

            };

        }(),

        rotateX: function () {

            var v1 = new THREE.Vector3( 1, 0, 0 );

            return function rotateX( angle ) {

                return this.rotateOnAxis( v1, angle );

            };

        }(),

        rotateY: function () {

            var v1 = new THREE.Vector3( 0, 1, 0 );

            return function rotateY( angle ) {

                return this.rotateOnAxis( v1, angle );

            };

        }(),

        rotateZ: function () {

            var v1 = new THREE.Vector3( 0, 0, 1 );

            return function rotateZ( angle ) {

                return this.rotateOnAxis( v1, angle );

            };

        }(),

        translateOnAxis: function () {

            // translate object by distance along axis in object space
            // axis is assumed to be normalized

            var v1 = new THREE.Vector3();

            return function translateOnAxis( axis, distance ) {

                v1.copy( axis ).applyQuaternion( this.quaternion );

                this.position.add( v1.multiplyScalar( distance ) );

                return this;

            };

        }(),

        translateX: function () {

            var v1 = new THREE.Vector3( 1, 0, 0 );

            return function translateX( distance ) {

                return this.translateOnAxis( v1, distance );

            };

        }(),

        translateY: function () {

            var v1 = new THREE.Vector3( 0, 1, 0 );

            return function translateY( distance ) {

                return this.translateOnAxis( v1, distance );

            };

        }(),

        translateZ: function () {

            var v1 = new THREE.Vector3( 0, 0, 1 );

            return function translateZ( distance ) {

                return this.translateOnAxis( v1, distance );

            };

        }(),

        localToWorld: function ( vector ) {

            return vector.applyMatrix4( this.matrixWorld );

        },

        worldToLocal: function () {

            var m1 = new THREE.Matrix4();

            return function worldToLocal( vector ) {

                return vector.applyMatrix4( m1.getInverse( this.matrixWorld ) );

            };

        }(),

        lookAt: function () {

            // This method does not support objects with rotated and/or translated parent(s)

            var m1 = new THREE.Matrix4();

            return function lookAt( vector ) {

                if ( this.isCamera ) {

                    m1.lookAt( this.position, vector, this.up );

                } else {

                    m1.lookAt( vector, this.position, this.up );

                }

                this.quaternion.setFromRotationMatrix( m1 );

            };

        }(),

        add: function ( object ) {

            if ( arguments.length > 1 ) {

                for ( var i = 0; i < arguments.length; i ++ ) {

                    this.add( arguments[ i ] );

                }

                return this;

            }

            if ( object === this ) {

                console.error( "THREE.Object3D.add: object can't be added as a child of itself.", object );
                return this;

            }

            if ( ( object && object.isObject3D ) ) {

                if ( object.parent !== null ) {

                    object.parent.remove( object );

                }

                object.parent = this;
                object.dispatchEvent( { type: 'added' } );

                this.children.push( object );

            } else {

                console.error( "THREE.Object3D.add: object not an instance of THREE.Object3D.", object );

            }

            return this;

        },

        remove: function ( object ) {

            if ( arguments.length > 1 ) {

                for ( var i = 0; i < arguments.length; i ++ ) {

                    this.remove( arguments[ i ] );

                }

                return this;

            }

            var index = this.children.indexOf( object );

            if ( index !== - 1 ) {

                object.parent = null;

                object.dispatchEvent( { type: 'removed' } );

                this.children.splice( index, 1 );

            }

            return this;

        },

        getObjectById: function ( id ) {

            return this.getObjectByProperty( 'id', id );

        },

        getObjectByName: function ( name ) {

            return this.getObjectByProperty( 'name', name );

        },

        getObjectByProperty: function ( name, value ) {

            if ( this[ name ] === value ) return this;

            for ( var i = 0, l = this.children.length; i < l; i ++ ) {

                var child = this.children[ i ];
                var object = child.getObjectByProperty( name, value );

                if ( object !== undefined ) {

                    return object;

                }

            }

            return undefined;

        },

        getWorldPosition: function ( optionalTarget ) {

            var result = optionalTarget || new THREE.Vector3();

            this.updateMatrixWorld( true );

            return result.setFromMatrixPosition( this.matrixWorld );

        },

        getWorldQuaternion: function () {

            var position = new THREE.Vector3();
            var scale = new THREE.Vector3();

            return function getWorldQuaternion( optionalTarget ) {

                var result = optionalTarget || new THREE.Quaternion();

                this.updateMatrixWorld( true );

                this.matrixWorld.decompose( position, result, scale );

                return result;

            };

        }(),

        getWorldRotation: function () {

            var quaternion = new THREE.Quaternion();

            return function getWorldRotation( optionalTarget ) {

                var result = optionalTarget || new THREE.Euler();

                this.getWorldQuaternion( quaternion );

                return result.setFromQuaternion( quaternion, this.rotation.order, false );

            };

        }(),

        getWorldScale: function () {

            var position = new THREE.Vector3();
            var quaternion = new THREE.Quaternion();

            return function getWorldScale( optionalTarget ) {

                var result = optionalTarget || new THREE.Vector3();

                this.updateMatrixWorld( true );

                this.matrixWorld.decompose( position, quaternion, result );

                return result;

            };

        }(),

        getWorldDirection: function () {

            var quaternion = new THREE.Quaternion();

            return function getWorldDirection( optionalTarget ) {

                var result = optionalTarget || new THREE.Vector3();

                this.getWorldQuaternion( quaternion );

                return result.set( 0, 0, 1 ).applyQuaternion( quaternion );

            };

        }(),

        raycast: function () {},

        traverse: function ( callback ) {

            callback( this );

            var children = this.children;

            for ( var i = 0, l = children.length; i < l; i ++ ) {

                children[ i ].traverse( callback );

            }

        },

        traverseVisible: function ( callback ) {

            if ( this.visible === false ) return;

            callback( this );

            var children = this.children;

            for ( var i = 0, l = children.length; i < l; i ++ ) {

                children[ i ].traverseVisible( callback );

            }

        },

        traverseAncestors: function ( callback ) {

            var parent = this.parent;

            if ( parent !== null ) {

                callback( parent );

                parent.traverseAncestors( callback );

            }

        },

        updateMatrix: function () {

            this.matrix.compose( this.position, this.quaternion, this.scale );

            this.matrixWorldNeedsUpdate = true;

        },

        updateMatrixWorld: function ( force ) {

            if ( this.matrixAutoUpdate ) this.updateMatrix();

            if ( this.matrixWorldNeedsUpdate || force ) {

                if ( this.parent === null ) {

                    this.matrixWorld.copy( this.matrix );

                } else {

                    this.matrixWorld.multiplyMatrices( this.parent.matrixWorld, this.matrix );

                }

                this.matrixWorldNeedsUpdate = false;

                force = true;

            }

            // update children

            var children = this.children;

            for ( var i = 0, l = children.length; i < l; i ++ ) {

                children[ i ].updateMatrixWorld( force );

            }

        },

        toJSON: function ( meta ) {

            // meta is '' when called from JSON.stringify
            var isRootObject = ( meta === undefined || meta === '' );

            var output = {};

            // meta is a hash used to collect geometries, materials.
            // not providing it implies that this is the root object
            // being serialized.
            if ( isRootObject ) {

                // initialize meta obj
                meta = {
                    geometries: {},
                    materials: {},
                    textures: {},
                    images: {}
                };

                output.metadata = {
                    version: 4.5,
                    type: 'Object',
                    generator: 'Object3D.toJSON'
                };

            }

            // standard Object3D serialization

            var object = {};

            object.uuid = this.uuid;
            object.type = this.type;

            if ( this.name !== '' ) object.name = this.name;
            if ( JSON.stringify( this.userData ) !== '{}' ) object.userData = this.userData;
            if ( this.castShadow === true ) object.castShadow = true;
            if ( this.receiveShadow === true ) object.receiveShadow = true;
            if ( this.visible === false ) object.visible = false;

            object.matrix = this.matrix.toArray();

            //

            function serialize( library, element ) {

                if ( library[ element.uuid ] === undefined ) {

                    library[ element.uuid ] = element.toJSON( meta );

                }

                return element.uuid;

            }

            if ( this.geometry !== undefined ) {

                object.geometry = serialize( meta.geometries, this.geometry );

            }

            if ( this.material !== undefined ) {

                if ( Array.isArray( this.material ) ) {

                    var uuids = [];

                    for ( var i = 0, l = this.material.length; i < l; i ++ ) {

                        uuids.push( serialize( meta.materials, this.material[ i ] ) );

                    }

                    object.material = uuids;

                } else {

                    object.material = serialize( meta.materials, this.material );

                }

            }

            //

            if ( this.children.length > 0 ) {

                object.children = [];

                for ( var i = 0; i < this.children.length; i ++ ) {

                    object.children.push( this.children[ i ].toJSON( meta ).object );

                }

            }

            if ( isRootObject ) {

                var geometries = extractFromCache( meta.geometries );
                var materials = extractFromCache( meta.materials );
                var textures = extractFromCache( meta.textures );
                var images = extractFromCache( meta.images );

                if ( geometries.length > 0 ) output.geometries = geometries;
                if ( materials.length > 0 ) output.materials = materials;
                if ( textures.length > 0 ) output.textures = textures;
                if ( images.length > 0 ) output.images = images;

            }

            output.object = object;

            return output;

            // extract data from the cache hash
            // remove metadata on each item
            // and return as array
            function extractFromCache( cache ) {

                var values = [];
                for ( var key in cache ) {

                    var data = cache[ key ];
                    delete data.metadata;
                    values.push( data );

                }
                return values;

            }

        },

        clone: function ( recursive ) {

            return new this.constructor().copy( this, recursive );

        },

        copy: function ( source, recursive ) {

            if ( recursive === undefined ) recursive = true;

            this.name = source.name;

            this.up.copy( source.up );

            this.position.copy( source.position );
            this.quaternion.copy( source.quaternion );
            this.scale.copy( source.scale );

            this.matrix.copy( source.matrix );
            this.matrixWorld.copy( source.matrixWorld );

            this.matrixAutoUpdate = source.matrixAutoUpdate;
            this.matrixWorldNeedsUpdate = source.matrixWorldNeedsUpdate;

            this.layers.mask = source.layers.mask;
            this.visible = source.visible;

            this.castShadow = source.castShadow;
            this.receiveShadow = source.receiveShadow;

            this.frustumCulled = source.frustumCulled;
            this.renderOrder = source.renderOrder;

            this.userData = JSON.parse( JSON.stringify( source.userData ) );

            if ( recursive === true ) {

                for ( var i = 0; i < source.children.length; i ++ ) {

                    var child = source.children[ i ];
                    this.add( child.clone() );

                }

            }

            return this;

        }

    } );


    //------CANNON------//


    /**
     * Dispatched after two bodies collide. This event is dispatched on each
     * of the two bodies involved in the collision.
     * @event collide
     * @param {Body} body The body that was involved in the collision.
     * @param {ContactEquation} contact The details of the collision.
     */
    TC.BodyObject3D.COLLIDE_EVENT_NAME = "collide";

    /**
     * A dynamic body is fully simulated. Can be moved manually by the user, but normally they move according to forces. A dynamic body can collide with all body types. A dynamic body always has finite, non-zero mass.
     * @static
     * @property DYNAMIC
     * @type {Number}
     */
    TC.BodyObject3D.DYNAMIC = 1;

    /**
     * A static body does not move during simulation and behaves as if it has infinite mass. Static bodies can be moved manually by setting the position of the body. The velocity of a static body is always zero. Static bodies do not collide with other static or kinematic bodies.
     * @static
     * @property STATIC
     * @type {Number}
     */
    TC.BodyObject3D.STATIC = 2;

    /**
     * A kinematic body moves under simulation according to its velocity. They do not respond to forces. They can be moved manually, but normally a kinematic body is moved by setting its velocity. A kinematic body behaves as if it has infinite mass. Kinematic bodies do not collide with other static or kinematic bodies.
     * @static
     * @property KINEMATIC
     * @type {Number}
     */
    TC.BodyObject3D.KINEMATIC = 4;



    /**
     * @static
     * @property AWAKE
     * @type {number}
     */
    TC.BodyObject3D.AWAKE = 0;

    /**
     * @static
     * @property SLEEPY
     * @type {number}
     */
    TC.BodyObject3D.SLEEPY = 1;

    /**
     * @static
     * @property SLEEPING
     * @type {number}
     */
    TC.BodyObject3D.SLEEPING = 2;


    /**
     * Dispatched after a sleeping body has woken up.
     * @event wakeup
     */
    TC.BodyObject3D.wakeupEvent = {
        type: "wakeup"
    };

    /**
     * Wake the body up.
     * @method wakeUp
     */
    TC.BodyObject3D.prototype.wakeUp = function(){
        var s = this.sleepState;
        this.sleepState = 0;
        this._wakeUpAfterNarrowphase = false;
        if(s === TC.BodyObject3D.SLEEPING){
            this.dispatchEvent(TC.BodyObject3D.wakeupEvent);
        }
    };

    /**
     * Force body sleep
     * @method sleep
     */
    TC.BodyObject3D.prototype.sleep = function(){
        this.sleepState = TC.BodyObject3D.SLEEPING;
        this.velocity.set(0,0,0);
        this.angularVelocity.set(0,0,0);
        this._wakeUpAfterNarrowphase = false;
    };

    /**
     * Dispatched after a body has gone in to the sleepy state.
     * @event sleepy
     */
    TC.BodyObject3D.sleepyEvent = {
        type: "sleepy"
    };

    /**
     * Dispatched after a body has fallen asleep.
     * @event sleep
     */
    TC.BodyObject3D.sleepEvent = {
        type: "sleep"
    };

    /**
     * Called every timestep to update internal sleep timer and change sleep state if needed.
     * @method sleepTick
     * @param {Number} time The world time in seconds
     */
    TC.BodyObject3D.prototype.sleepTick = function(time){
        if(this.allowSleep){
            var sleepState = this.sleepState;
            var speedSquared = this.velocity.norm2() + this.angularVelocity.norm2();
            var speedLimitSquared = Math.pow(this.sleepSpeedLimit,2);
            if(sleepState===TC.BodyObject3D.AWAKE && speedSquared < speedLimitSquared){
                this.sleepState = TC.BodyObject3D.SLEEPY; // Sleepy
                this.timeLastSleepy = time;
                this.dispatchEvent(TC.BodyObject3D.sleepyEvent);
            } else if(sleepState===TC.BodyObject3D.SLEEPY && speedSquared > speedLimitSquared){
                this.wakeUp(); // Wake up
            } else if(sleepState===TC.BodyObject3D.SLEEPY && (time - this.timeLastSleepy ) > this.sleepTimeLimit){
                this.sleep(); // Sleeping
                this.dispatchEvent(TC.BodyObject3D.sleepEvent);
            }
        }
    };

    /**
     * If the body is sleeping, it should be immovable / have infinite mass during solve. We solve it by having a separate "solve mass".
     * @method updateSolveMassProperties
     */
    TC.BodyObject3D.prototype.updateSolveMassProperties = function(){
        if(this.sleepState === TC.BodyObject3D.SLEEPING || this.type === TC.BodyObject3D.KINEMATIC){
            this.invMassSolve = 0;
            this.invInertiaSolve.setZero();
            this.invInertiaWorldSolve.setZero();
        } else {
            this.invMassSolve = this.invMass;
            this.invInertiaSolve.copy(this.invInertia);
            this.invInertiaWorldSolve.copy(this.invInertiaWorld);
        }
    };

    /**
     * Convert a world point to local body frame.
     * @method pointToLocalFrame
     * @param  {Vec3} worldPoint
     * @param  {Vec3} result
     * @return {Vec3}
     */
    TC.BodyObject3D.prototype.pointToLocalFrame = function(worldPoint,result){
        var result = result || new CANNON.Vec3();
        worldPoint.vsub(this.position,result);
        this.quaternion.conjugate().vmult(result,result);
        return result;
    };

    /**
     * Convert a world vector to local body frame.
     * @method vectorToLocalFrame
     * @param  {Vec3} worldPoint
     * @param  {Vec3} result
     * @return {Vec3}
     */
    TC.BodyObject3D.prototype.vectorToLocalFrame = function(worldVector, result){
        var result = result || new CANNON.Vec3();
        this.quaternion.conjugate().vmult(worldVector,result);
        return result;
    };

    /**
     * Convert a local body point to world frame.
     * @method pointToWorldFrame
     * @param  {Vec3} localPoint
     * @param  {Vec3} result
     * @return {Vec3}
     */
    TC.BodyObject3D.prototype.pointToWorldFrame = function(localPoint,result){
        var result = result || new CANNON.Vec3();
        this.quaternion.vmult(localPoint,result);
        result.vadd(this.position,result);
        return result;
    };

    /**
     * Convert a local body point to world frame.
     * @method vectorToWorldFrame
     * @param  {Vec3} localVector
     * @param  {Vec3} result
     * @return {Vec3}
     */
    TC.BodyObject3D.prototype.vectorToWorldFrame = function(localVector, result){
        var result = result || new CANNON.Vec3();
        this.quaternion.vmult(localVector, result);
        return result;
    };

    var tmpVec = new CANNON.Vec3();
    var tmpQuat = new THREE.Quaternion();

    /**
     * Add a shape to the body with a local offset and orientation.
     * @method addShape
     * @param {Shape} shape
     * @param {Vec3} [_offset]
     * @param {Quaternion} [_orientation]
     * @return {TC.BodyObject3D} The body object, for chainability.
     */
    TC.BodyObject3D.prototype.addShape = function(shape, _offset, _orientation){
        var offset = new CANNON.Vec3();
        var orientation = new THREE.Quaternion();

        if(_offset){
            offset.copy(_offset);
        }
        if(_orientation){
            orientation.copy(_orientation);
        }

        this.shapes.push(shape);
        this.shapeOffsets.push(offset);
        this.shapeOrientations.push(orientation);
        this.updateMassProperties();
        this.updateBoundingRadius();

        this.aabbNeedsUpdate = true;

        shape.body = this;

        return this;
    };

    /**
     * Update the bounding radius of the body. Should be done if any of the shapes are changed.
     * @method updateBoundingRadius
     */
    TC.BodyObject3D.prototype.updateBoundingRadius = function(){
        var shapes = this.shapes,
            shapeOffsets = this.shapeOffsets,
            N = shapes.length,
            radius = 0;

        for(var i=0; i!==N; i++){
            var shape = shapes[i];
            shape.updateBoundingSphereRadius();
            var offset = shapeOffsets[i].norm(),
                r = shape.boundingSphereRadius;
            if(offset + r > radius){
                radius = offset + r;
            }
        }

        this.boundingRadius = radius;
    };

    var computeAABB_shapeAABB = new CANNON.AABB();

    /**
     * Updates the .aabb
     * @method computeAABB
     * @todo rename to updateAABB()
     */
    TC.BodyObject3D.prototype.computeAABB = function(){
        var shapes = this.shapes,
            shapeOffsets = this.shapeOffsets,
            shapeOrientations = this.shapeOrientations,
            N = shapes.length,
            offset = tmpVec,
            orientation = tmpQuat,
            bodyQuat = this.quaternion,
            aabb = this.aabb,
            shapeAABB = computeAABB_shapeAABB;

        for(var i=0; i!==N; i++){
            var shape = shapes[i];

            // Get shape world position
            bodyQuat.vmult(shapeOffsets[i], offset);
            offset.vadd(this.position, offset);

            // Get shape world quaternion
            shapeOrientations[i].mult(bodyQuat, orientation);

            // Get shape AABB
            shape.calculateWorldAABB(offset, orientation, shapeAABB.lowerBound, shapeAABB.upperBound);

            if(i === 0){
                aabb.copy(shapeAABB);
            } else {
                aabb.extend(shapeAABB);
            }
        }

        this.aabbNeedsUpdate = false;
    };

    var uiw_m1 = new CANNON.Mat3(),
        uiw_m2 = new CANNON.Mat3(),
        uiw_m3 = new CANNON.Mat3();

    /**
     * Update .inertiaWorld and .invInertiaWorld
     * @method updateInertiaWorld
     */
    TC.BodyObject3D.prototype.updateInertiaWorld = function(force){
        var I = this.invInertia;
        if (I.x === I.y && I.y === I.z && !force) {
            // If inertia M = s*I, where I is identity and s a scalar, then
            //    R*M*R' = R*(s*I)*R' = s*R*I*R' = s*R*R' = s*I = M
            // where R is the rotation matrix.
            // In other words, we don't have to transform the inertia if all
            // inertia diagonal entries are equal.
        } else {
            var m1 = uiw_m1,
                m2 = uiw_m2,
                m3 = uiw_m3;
            m1.setRotationFromQuaternion(this.quaternion);
            m1.transpose(m2);
            m1.scale(I,m1);
            m1.mmult(m2,this.invInertiaWorld);
        }
    };

    /**
     * Apply force to a world point. This could for example be a point on the Body surface. Applying force this way will add to TC.BodyObject3D.force and TC.BodyObject3D.torque.
     * @method applyForce
     * @param  {Vec3} force The amount of force to add.
     * @param  {Vec3} relativePoint A point relative to the center of mass to apply the force on.
     */
    var Body_applyForce_r = new CANNON.Vec3();
    var Body_applyForce_rotForce = new CANNON.Vec3();
    TC.BodyObject3D.prototype.applyForce = function(force,relativePoint){
        if(this.type !== TC.BodyObject3D.DYNAMIC){ // Needed?
            return;
        }

        // Compute produced rotational force
        var rotForce = Body_applyForce_rotForce;
        relativePoint.cross(force,rotForce);

        // Add linear force
        this.force.vadd(force,this.force);

        // Add rotational force
        this.torque.vadd(rotForce,this.torque);
    };

    /**
     * Apply force to a local point in the body.
     * @method applyLocalForce
     * @param  {Vec3} force The force vector to apply, defined locally in the body frame.
     * @param  {Vec3} localPoint A local point in the body to apply the force on.
     */
    var Body_applyLocalForce_worldForce = new CANNON.Vec3();
    var Body_applyLocalForce_relativePointWorld = new CANNON.Vec3();
    TC.BodyObject3D.prototype.applyLocalForce = function(localForce, localPoint){
        if(this.type !== TC.BodyObject3D.DYNAMIC){
            return;
        }

        var worldForce = Body_applyLocalForce_worldForce;
        var relativePointWorld = Body_applyLocalForce_relativePointWorld;

        // Transform the force vector to world space
        this.vectorToWorldFrame(localForce, worldForce);
        this.vectorToWorldFrame(localPoint, relativePointWorld);

        this.applyForce(worldForce, relativePointWorld);
    };

    /**
     * Apply impulse to a world point. This could for example be a point on the Body surface. An impulse is a force added to a body during a short period of time (impulse = force * time). Impulses will be added to TC.BodyObject3D.velocity and TC.BodyObject3D.angularVelocity.
     * @method applyImpulse
     * @param  {Vec3} impulse The amount of impulse to add.
     * @param  {Vec3} relativePoint A point relative to the center of mass to apply the force on.
     */
    var Body_applyImpulse_r = new CANNON.Vec3();
    var Body_applyImpulse_velo = new CANNON.Vec3();
    var Body_applyImpulse_rotVelo = new CANNON.Vec3();
    TC.BodyObject3D.prototype.applyImpulse = function(impulse, relativePoint){
        if(this.type !== TC.BodyObject3D.DYNAMIC){
            return;
        }

        // Compute point position relative to the body center
        var r = relativePoint;

        // Compute produced central impulse velocity
        var velo = Body_applyImpulse_velo;
        velo.copy(impulse);
        velo.mult(this.invMass,velo);

        // Add linear impulse
        this.velocity.vadd(velo, this.velocity);

        // Compute produced rotational impulse velocity
        var rotVelo = Body_applyImpulse_rotVelo;
        r.cross(impulse,rotVelo);

        /*
         rotVelo.x *= this.invInertia.x;
         rotVelo.y *= this.invInertia.y;
         rotVelo.z *= this.invInertia.z;
         */
        this.invInertiaWorld.vmult(rotVelo,rotVelo);

        // Add rotational Impulse
        this.angularVelocity.vadd(rotVelo, this.angularVelocity);
    };

    /**
     * Apply locally-defined impulse to a local point in the body.
     * @method applyLocalImpulse
     * @param  {Vec3} force The force vector to apply, defined locally in the body frame.
     * @param  {Vec3} localPoint A local point in the body to apply the force on.
     */
    var Body_applyLocalImpulse_worldImpulse = new CANNON.Vec3();
    var Body_applyLocalImpulse_relativePoint = new CANNON.Vec3();
    TC.BodyObject3D.prototype.applyLocalImpulse = function(localImpulse, localPoint){
        if(this.type !== TC.BodyObject3D.DYNAMIC){
            return;
        }

        var worldImpulse = Body_applyLocalImpulse_worldImpulse;
        var relativePointWorld = Body_applyLocalImpulse_relativePoint;

        // Transform the force vector to world space
        this.vectorToWorldFrame(localImpulse, worldImpulse);
        this.vectorToWorldFrame(localPoint, relativePointWorld);

        this.applyImpulse(worldImpulse, relativePointWorld);
    };

    var Body_updateMassProperties_halfExtents = new CANNON.Vec3();

    /**
     * Should be called whenever you change the body shape or mass.
     * @method updateMassProperties
     */
    TC.BodyObject3D.prototype.updateMassProperties = function(){
        var halfExtents = Body_updateMassProperties_halfExtents;

        this.invMass = this.mass > 0 ? 1.0 / this.mass : 0;
        var I = this.inertia;
        var fixed = this.fixedRotation;

        // Approximate with AABB box
        this.computeAABB();
        halfExtents.set(
            (this.aabb.upperBound.x-this.aabb.lowerBound.x) / 2,
            (this.aabb.upperBound.y-this.aabb.lowerBound.y) / 2,
            (this.aabb.upperBound.z-this.aabb.lowerBound.z) / 2
        );
        CANNON.Box.calculateInertia(halfExtents, this.mass, I);

        this.invInertia.set(
            I.x > 0 && !fixed ? 1.0 / I.x : 0,
            I.y > 0 && !fixed ? 1.0 / I.y : 0,
            I.z > 0 && !fixed ? 1.0 / I.z : 0
        );
        this.updateInertiaWorld(true);
    };

    /**
     * Get world velocity of a point in the body.
     * @method getVelocityAtWorldPoint
     * @param  {Vec3} worldPoint
     * @param  {Vec3} result
     * @return {Vec3} The result vector.
     */
    TC.BodyObject3D.prototype.getVelocityAtWorldPoint = function(worldPoint, result){
        var r = new CANNON.Vec3();
        worldPoint.vsub(this.position, r);
        this.angularVelocity.cross(r, result);
        this.velocity.vadd(result, result);
        return result;
    };

    var torque = new CANNON.Vec3();
    var invI_tau_dt = new CANNON.Vec3();
    var w = new THREE.Quaternion();
    var wq = new THREE.Quaternion();

    /**
     * Move the body forward in time.
     * @param {number} dt Time step
     * @param {boolean} quatNormalize Set to true to normalize the body quaternion
     * @param {boolean} quatNormalizeFast If the quaternion should be normalized using "fast" quaternion normalization
     */
    TC.BodyObject3D.prototype.integrate = function(dt, quatNormalize, quatNormalizeFast){

        // Save previous position
        this.previousPosition.copy(this.position);
        this.previousQuaternion.copy(this.quaternion);

        if(!(this.type === TC.BodyObject3D.DYNAMIC || this.type === TC.BodyObject3D.KINEMATIC) || this.sleepState === TC.BodyObject3D.SLEEPING){ // Only for dynamic
            return;
        }

        var velo = this.velocity,
            angularVelo = this.angularVelocity,
            pos = this.position,
            force = this.force,
            torque = this.torque,
            quat = this.quaternion,
            invMass = this.invMass,
            invInertia = this.invInertiaWorld,
            linearFactor = this.linearFactor;

        var iMdt = invMass * dt;
        velo.x += force.x * iMdt * linearFactor.x;
        velo.y += force.y * iMdt * linearFactor.y;
        velo.z += force.z * iMdt * linearFactor.z;

        var e = invInertia.elements;
        var angularFactor = this.angularFactor;
        var tx = torque.x * angularFactor.x;
        var ty = torque.y * angularFactor.y;
        var tz = torque.z * angularFactor.z;
        angularVelo.x += dt * (e[0] * tx + e[1] * ty + e[2] * tz);
        angularVelo.y += dt * (e[3] * tx + e[4] * ty + e[5] * tz);
        angularVelo.z += dt * (e[6] * tx + e[7] * ty + e[8] * tz);

        // Use new velocity  - leap frog
        pos.x += velo.x * dt;
        pos.y += velo.y * dt;
        pos.z += velo.z * dt;

        quat.integrate(this.angularVelocity, dt, this.angularFactor, quat);

        if(quatNormalize){
            if(quatNormalizeFast){
                quat.normalizeFast();
            } else {
                quat.normalize();
            }
        }

        this.aabbNeedsUpdate = true;

        // Update world inertia
        this.updateInertiaWorld();
    };

})(window.TC);
