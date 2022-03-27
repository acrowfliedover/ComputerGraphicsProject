import {tiny, defs} from './common.js';

                                                  // Pull these names into this module's scope for convenience:
const { Vec, Mat, Mat4, Color, Light, Shape, Material, Shader, Texture, Scene } = tiny;

var HelperMethods = {
  dot: function(v, u) {
    return u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
  },

  length: function(v) {
    return Math.sqrt(this.dot(v, v));
  }
}

export class Body
{                                   // **Body** can store and update the properties of a 3D body that incrementally
                                    // moves from its previous place due to velocities.  It conforms to the
                                    // approach outlined in the "Fix Your Timestep!" blog post by Glenn Fiedler.
  constructor( shape, material, size )
    { Object.assign( this,
             { shape, material, size } )
    }
  emplace( location_matrix, linear_velocity, angular_velocity, spin_axis = Vec.of(0,0,0).randomized(1).normalized() )
    {                               // emplace(): assign the body's initial values, or overwrite them.
      this.center   = location_matrix.times( Vec.of( 0,0,0,1 ) ).to3();
      this.rotation = Mat4.translation( this.center.times( -1 ) ).times( location_matrix );
      this.previous = { center: this.center.copy(), rotation: this.rotation.copy() };
                                           // drawn_location gets replaced with an interpolated quantity:
      this.drawn_location = location_matrix;
      return Object.assign( this, { linear_velocity, angular_velocity, spin_axis } )
    }
  advance( time_amount )
    {                           // advance(): Perform an integration (the simplistic Forward Euler method) to
                                // advance all the linear and angular velocities one time-step forward.
      this.previous = { center: this.center.copy(), rotation: this.rotation.copy() };
                                                 // Apply the velocities scaled proportionally to real time (time_amount):
                                                 // Linear velocity first, then angular:
      this.center = this.center.plus( this.linear_velocity.times( time_amount ) );
      this.rotation.pre_multiply( Mat4.rotation( time_amount * this.angular_velocity, this.spin_axis ) );
    }
  blend_rotation( alpha )
    {                        // blend_rotation(): Just naively do a linear blend of the rotations, which looks
                             // ok sometimes but otherwise produces shear matrices, a wrong result.

                                  // TODO:  Replace this function with proper quaternion blending, and perhaps
                                  // store this.rotation in quaternion form instead for compactness.
       return this.rotation.map( (x,i) => Vec.from( this.previous.rotation[i] ).mix( x, alpha ) );
    }
  blend_state( alpha )
    {                             // blend_state(): Compute the final matrix we'll draw using the previous two physical
                                  // locations the object occupied.  We'll interpolate between these two states as
                                  // described at the end of the "Fix Your Timestep!" blog post.
      this.drawn_location = Mat4.translation( this.previous.center.mix( this.center, alpha ) )
                                      .times( this.blend_rotation( alpha ) )
                                      .times( Mat4.scale( this.size ) );
    }
                                              // The following are our various functions for testing a single point,
                                              // p, against some analytically-known geometric volume formula
                                              // (within some margin of distance).
  static intersect_cube( p, margin = 0 )
    { return p.every( value => value >= -1 - margin && value <=  1 + margin )
    }
  static intersect_sphere( p, margin = 0 )
    { return p.dot( p ) < 1 + margin;
    }
  check_if_colliding( b, collider )
    {                                     // check_if_colliding(): Collision detection function.
                                          // DISCLAIMER:  The collision method shown below is not used by anyone; it's just very quick
                                          // to code.  Making every collision body an ellipsoid is kind of a hack, and looping
                                          // through a list of discrete sphere points to see if the ellipsoids intersect is *really* a
                                          // hack (there are perfectly good analytic expressions that can test if two ellipsoids
                                          // intersect without discretizing them into points).
      if ( this == b )
        return false;                     // Nothing collides with itself.
                                          // Convert sphere b to the frame where a is a unit sphere:
      var T = this.inverse.times( b.drawn_location );

      const { intersect_test, points, leeway } = collider;
                                          // For each vertex in that b, shift to the coordinate frame of
                                          // a_inv*b.  Check if in that coordinate frame it penetrates
                                          // the unit sphere at the origin.  Leave some leeway.
      return points.arrays.position.some( p => intersect_test( T.times( p.to4(1) ).to3(), leeway ) );
    }
}


export class Simulation extends Scene
{                                         // **Simulation** manages the stepping of simulation time.  Subclass it when making
                                          // a Scene that is a physics demo.  This technique is careful to totally decouple
                                          // the simulation from the frame rate (see below).
  constructor()
    { super();
      Object.assign( this, { time_accumulator: 0, time_scale: 1, t: 0, dt: 1/20, bodies: [], steps_taken: 0 } );
    }
  simulate( frame_time )
    {                                     // simulate(): Carefully advance time according to Glenn Fiedler's
                                          // "Fix Your Timestep" blog post.
                                          // This line gives ourselves a way to trick the simulator into thinking
                                          // that the display framerate is running fast or slow:
      frame_time = this.time_scale * frame_time;

                                          // Avoid the spiral of death; limit the amount of time we will spend
                                          // computing during this timestep if display lags:
      this.time_accumulator += Math.min( frame_time, 0.1 );
                                          // Repeatedly step the simulation until we're caught up with this frame:
      while ( Math.abs( this.time_accumulator ) >= this.dt )
      {                                                       // Single step of the simulation for all bodies:
        this.update_state( this.dt  );
        for( let b of this.bodies )
          b.advance( this.dt );
                                          // Following the advice of the article, de-couple
                                          // our simulation time from our frame rate:
        this.t                += Math.sign( frame_time ) * this.dt;
        this.time_accumulator -= Math.sign( frame_time ) * this.dt;
        this.steps_taken++;
      }
                                            // Store an interpolation factor for how close our frame fell in between
                                            // the two latest simulation time steps, so we can correctly blend the
                                            // two latest states and display the result.
      let alpha = this.time_accumulator / this.dt;
      for( let b of this.bodies ) b.blend_state( alpha );
    }
  make_control_panel(program_state)
    {                       // make_control_panel(): Create the buttons for interacting with simulation time.
      this.key_triggered_button( "Speed up time", [ "Shift","T" ], () => this.time_scale *= 5           );
      this.key_triggered_button( "Slow down time",        [ "t" ], () => this.time_scale /= 5           ); this.new_line();
      this.live_string( box => { box.textContent = "Time scale: "  + this.time_scale                  } ); this.new_line();
      this.live_string( box => { box.textContent = "Fixed simulation time step size: "  + this.dt     } ); this.new_line();
      this.live_string( box => { box.textContent = this.steps_taken + " timesteps were taken so far." } );
      this.key_triggered_button( "Right Flipper", ["h"], () => {this.left_flag = true; this.left_value = this.steps_taken;});
      this.key_triggered_button( "Left Flipper", ["g"], () => {this.right_flag = true; this.right_value = this.steps_taken;});
    }
  display( context, program_state )
    {                                     // display(): advance the time and state of our whole simulation.
      if(program_state.needs_reset == true){
        location.reload();
      }
      if( program_state.animate )
        this.simulate( program_state.animation_delta_time );
                                          // Draw each shape at its current location:
      for( let b of this.bodies )
        b.shape.draw( context, program_state, b.drawn_location, b.material );
    }
  update_state( dt )      // update_state(): Your subclass of Simulation has to override this abstract function.
    { throw "Override this" }
}


export class Test_Data
{                             // **Test_Data** pre-loads some Shapes and Textures that other Scenes can borrow.
  constructor()
    { this.textures = { rgb   : new Texture( "assets/rgb.jpg" ),
                        earth : new Texture( "assets/earth.gif" ),
                        grid  : new Texture( "assets/grid.png" ),
                        stars : new Texture( "assets/stars.png" ),
                        text  : new Texture( "assets/text.png" ),
                      }
      this.shapes = { donut  : new defs.Torus          ( 15, 15, [[0,2],[0,1]] ),
                      cone   : new defs.Closed_Cone    ( 4, 10,  [[0,2],[0,1]] ),
                      capped : new defs.Capped_Cylinder( 4, 12,  [[0,2],[0,1]] ),
                      ball_6   : new defs.Subdivision_Sphere( 6, [[0,1],[0,1]]),
                      arc    : new  defs.Cylindrical_Sheet( 10, 10, Math.PI/2, [[0,2],[0,1]]),
//defs.Part_Torus(15, 15, [[0,2],[0,1]],Math.Pi),
                      cube   : new defs.Cube(),
                      prism  : new ( defs.Capped_Cylinder   .prototype.make_flat_shaded_version() )( 10, 10, [[0,2],[0,1]] ),
                      gem    : new ( defs.Subdivision_Sphere.prototype.make_flat_shaded_version() )( 2 ),
                      donut  : new ( defs.Torus             .prototype.make_flat_shaded_version() )( 20, 20, [[0,5],[0,0.5]] ),

                    };
    }
  random_shape( shape_list = this.shapes )
    {                                       // random_shape():  Extract a random shape from this.shapes.
      const shape_names = Object.keys( shape_list );
      return shape_list[ shape_names[ ~~( shape_names.length * Math.random() ) ] ]
    }
}













export class Inertia_Demo extends Simulation
{                                           // ** Inertia_Demo** demonstration: This scene lets random initial momentums
                                            // carry several bodies until they fall due to gravity and bounce.
  constructor()
    { super();
      this.data = new Test_Data();
      this.shapes = Object.assign( {}, this.data.shapes );
      this.shapes.square = new defs.Square();
      const shader = new defs.Fake_Bump_Map( 1 );
      this.left_value = 0;
      this.right_value = 0;
      this.left_flag = false;
      this.right_flag = false;
      this.m_score = 0;
      this.m_lives = 3;

      this.left_flipper_rotation = 0;
      this.right_flipper_rotation = 0;
      this.left_flipper_center = [0,0,0];
      this.right_flipper_center = [0,0,0];

      this.collision_right_rectangle = false;
      this.collision_left_rectangle = false;
      this.collision_right_rectangle_flipper = false;
      this.collision_left_rectangle_flipper = false;

      this.collision_right_rectangle_iterations = 0;
      this.collision_left_rectangle_iterations = 0;
      this.collision_right_rectangle_flipper_iterations = 0;
      this.collision_left_rectangle_flipper_iterations = 0;

      const phong_shader      = new defs.Phong_Shader  (2);
                                                              // Adding textures to the previous shader:
      const texture_shader    = new defs.Textured_Phong(2);
      const texture_shader2 = new defs.Fake_Bump_Map( 2 );
      const black_hole_shader = new Sun_Shader();

      this.material = new Material( phong_shader, { color: Color.of( .4,.8,.4,1 ),
                                  ambient:.4})
      this.material.background = new Material(texture_shader2, {texture: new Texture("assets/shiny.jpg"),  ambient: 0.4, color: Color.of( .4,.8,.4,1) } );
      this.material.blocks = new Material(texture_shader2, {texture: new Texture("assets/colorful.jpg"), ambient: 0.4, color: Color.of(0.3, 0.3, 0.3, 1) } );
      this.material.levers = new Material(texture_shader2, {texture: new Texture("assets/vapor.jpg"), ambient: 0.4, color: Color.of(0.5, 0.5, 0.5, 1) } );
      this.material.donuts = new Material(phong_shader, { ambient: 0.4, color: Color.of(0, 0, 1, 1) } );
      this.material.cone = new Material(phong_shader, { ambient: 0.4, color: Color.of(0, 0.8, 1, 1) } );
      this.material.other = new Material( phong_shader, { color:  Color.of( 1,1,0,1 ), ambient:.4});
      this.material.snake = new Material(texture_shader2, {texture: new Texture("assets/snake.jpg"),ambient: 0.4, color: Color.of(251/255, 68/255, 181/255, 1) } );
      this.material.cylinder1 = new Material(texture_shader2, {texture: new Texture("assets/purple.jpg"),  ambient: 0.4, color: Color.of(1, 0, 0, 1) } );
      this.material.cylinder2 = new Material(texture_shader2, {texture: new Texture("assets/purple.jpg"),  ambient: 0.4, color: Color.of(123/255, 104/255, 238/255, 1) } );
      this.material.cylinder3 = new Material(texture_shader2, {texture: new Texture("assets/purple.jpg"),  ambient: 0.4, color: Color.of(1, 215/255, 0, 1) } );
      this.material.black_hole = new Material(black_hole_shader, { ambient: 1, color: Color.of( 1,.5,1,1 ) } );
    }
  update_state( dt )
    {                 // update_state():  Override the base time-stepping code to say what this particular
                      // scene should do to its bodies every frame -- including applying forces.
                      // Generate additional moving bodies if there ever aren't enough:
      if(this.m_lives <= 0){
        return;
      }
      while( this.bodies.length <1 )
        this.bodies.push( new Body( this.shapes.ball_6, this.material.snake, Vec.of( 1,1,1 ) )
                .emplace( Mat4.translation( Vec.of(14, -15,1) ),
                          Vec.of(0,1.2,0).normalized().times(15), Math.random() ) );


//       this.flippers.push(this.shapes.cube.draw(context, program_state, model_transform.times(Mat4.rotation(Math.PI/4, Vec.of(0,0,1))).times(Mat4.translation([-7,-15,1])).times(Mat4.scale([4,1,1])), this.material.blocks );)
//       this.flippers.push( new Body (this.shapes.cube, this.material.blocks, Vec) );

//       this.shapes.cube.draw(context, program_state, model_transform.times(Mat4.rotation(Math.PI/4, Vec.of(0,0,1))).times(Mat4.translation([-7,-15,1])).times(Mat4.scale([4,1,1])), this.material.blocks );
//       this.shapes.cube.draw( context, program_state, model_transform.times(Mat4.rotation(Math.PI*0.75, Vec.of(0,0,1))).times(Mat4.translation([-7,15,1])).times(Mat4.scale([4,1,1])), this.material.blocks );

      for( let b of this.bodies )
      {                                         // Gravity on Earth, where 1 unit in world space = 1 meter:
        b.linear_velocity[1] += dt * -1.0;

        b.linear_velocity[1]*=.999;
      }
      //////////////////////////////////////////////////////////
      // for cirular objects
      //////////////////////////////////////////////////////////
      this.circular_obs_center = new Array()
      this.circular_obs_radius = new Array()
      this.circular_obs_center.push([0, 5, 0])
      this.circular_obs_radius.push(1)
      this.circular_obs_center.push([-3, 2, 0])
      this.circular_obs_radius.push(1)
      this.circular_obs_center.push([3, 2, 0])
      this.circular_obs_radius.push(1)

      let i = 0

      while (i < this.circular_obs_radius.length){
        if (Math.sqrt((this.bodies[0].center[0] - this.circular_obs_center[i][0])**2 + (this.bodies[0].center[1] - this.circular_obs_center[i][1])**2) <= this.circular_obs_radius[i]+1) {
            let normal_vec = Vec.of(this.bodies[0].center[0]-this.circular_obs_center[i][0],this.bodies[0].center[1] - this.circular_obs_center[i][1] ,0).normalized();
            let cos_theta = HelperMethods.dot( this.bodies[0].linear_velocity, normal_vec) /  HelperMethods.length(this.bodies[0].linear_velocity);
            let theta = Math.acos(cos_theta);
            this.bodies[0].linear_velocity = this.bodies[0].linear_velocity.plus(normal_vec.times((HelperMethods.length(this.bodies[0].linear_velocity) *(-Math.abs (cos_theta))) * (-1.95)  ));
            this.m_score += 100;
        }
        i += 1
      }

      //////////////////////////////////////////////////////////
      // for cirular objects [THE END]
      //////////////////////////////////////////////////////////


      //////////////////////////////////////////////////////////
      // for arcs //assuming all arcs are 1/4 circles
      //////////////////////////////////////////////////////////
      this.arc_center=new Array();
      this.arc_radius=new Array();
      this.arc_rotation= new Array();
     // this.arc_center.push([0,-30,1]);
     // this.arc_center.push([0,-5,1])
     //  this.arc_radius.push(10);
     //  this.arc_radius.push(10);
     // this.arc_rotation.push(Math.PI/4);
     //this.arc_rotation.push(-Math.PI/2);
     if(this.steps_taken>=130){
      this.arc_center.push([ 6.2,-6-10,1],[ -6.2,-6-10,1]);
       this.arc_radius.push(6.3,6.3);
       this.arc_rotation.push(-Math.PI/2.5,Math.PI/2+Math.PI/2.5);
     }
       if(this.steps_taken<130){ //I want to use program_state.time here

         this.arc_center.push([8,3,1],[6,1,1]);
         this.arc_radius.push(7,6);
         this.arc_rotation.push(0,0);
       }

      for (i=0; i<this.arc_radius.length; ++i){

        let rotated_center_x=(this.bodies[0].center[0]-this.arc_center[i][0])*(Math.cos(-this.arc_rotation[i]))-(this.bodies[0].center[1]-this.arc_center[i][1])*Math.sin(-this.arc_rotation[i]); //if we rotate the ball back
        let rotated_center_y=(this.bodies[0].center[1]-this.arc_center[i][1])*(Math.cos(-this.arc_rotation[i]))+(this.bodies[0].center[0]-this.arc_center[i][0])*Math.sin(-this.arc_rotation[i]);

        let rotated_top_x=this.arc_center[i][0]-this.arc_radius[i]*Math.sin(this.arc_rotation[i]);//top of arc after rotation
        let rotated_top_y=this.arc_center[i][1]+this.arc_radius[i]*Math.cos(this.arc_rotation[i]);

        let rotated_right_x=this.arc_center[i][0]+this.arc_radius[i]*Math.cos(this.arc_rotation[i]);//right of arc after rotation
        let rotated_right_y=this.arc_center[i][1]+this.arc_radius[i]*Math.sin(this.arc_rotation[i]);

        let distance_to_center=Math.sqrt((this.bodies[0].center[0] - this.arc_center[i][0])**2 + (this.bodies[0].center[1] - this.arc_center[i][1])**2);

        if ((distance_to_center <= this.arc_radius[i]+1  && //outer boundary
             distance_to_center >= this.arc_radius[i]+0.5) ||
             (distance_to_center <= this.arc_radius[i]-0.5  && //inner boundary
             distance_to_center >= this.arc_radius[i]-1) ){
   //  console.log(rotated_center_x,rotated_center_y);
//        console.log(distance_to_center);
             if(rotated_center_x > 0 && //in the first quadrant
                rotated_center_y > 0 ){
              // console.log("bounce");
                let normal_vec=Vec.of(0,0,1);

                    if(distance_to_center>this.arc_radius[i]){

                        normal_vec = Vec.of(this.bodies[0].center[0]-this.arc_center[i][0],this.bodies[0].center[1]-this.arc_center[i][1] ,0).normalized();
                    }
                    else{

                        normal_vec = Vec.of(-this.bodies[0].center[0]+this.arc_center[i][0],-this.bodies[0].center[1] +this.arc_center[i][1] ,0).normalized();
                    }
                let cos_theta = HelperMethods.dot( this.bodies[0].linear_velocity, normal_vec) / HelperMethods.length(this.bodies[0].linear_velocity);
                this.bodies[0].linear_velocity = this.bodies[0].linear_velocity.plus(normal_vec.times((HelperMethods.length(this.bodies[0].linear_velocity) * (-Math.abs (cos_theta))) * (-1))  );
             //console.log(normal_vec);
             }
             else if
             (Math.sqrt((rotated_center_y-this.arc_radius[i])**2 + //y of top tip of arc
             (rotated_center_x)**2) <= 1 ){//x of top tip of arc
             //console.log("top")
                let normal_vec = Vec.of(this.bodies[0].center[0]-rotated_top_x,this.bodies[0].center[1] -rotated_top_y ,0).normalized();
                let cos_theta = HelperMethods.dot( this.bodies[0].linear_velocity, normal_vec) / HelperMethods.length(this.bodies[0].linear_velocity);
                this.bodies[0].linear_velocity = this.bodies[0].linear_velocity.plus(normal_vec.times((HelperMethods.length(this.bodies[0].linear_velocity) * (-Math.abs (cos_theta))) * (-1.7)))  ;
             }
             else if(Math.sqrt(rotated_center_y**2 + //y of right tip of arc
             ((rotated_center_x-this.arc_radius[i])**2) <= 1)){ //x of right tip of arc
             //console.log("right");
                let normal_vec = Vec.of(this.bodies[0].center[0]-rotated_right_x,this.bodies[0].center[1] -rotated_right_y ,0).normalized();
                let cos_theta = HelperMethods.dot( this.bodies[0].linear_velocity, normal_vec) /  HelperMethods.length(this.bodies[0].linear_velocity);
                this.bodies[0].linear_velocity = this.bodies[0].linear_velocity.plus(normal_vec.times((HelperMethods.length(this.bodies[0].linear_velocity) * (-Math.abs (cos_theta))) * (-1.7) ) );
             }
         }
      }
      //////////////////////////////////////////////////////////
      //arcs done
      //////////////////////////////////////////////////////////



      //////////////////////////////////////////////////////////
      // for boundaries
      //////////////////////////////////////////////////////////
      let upper_bound_center = [0, 11, 0];
      let upper_bound_width = 1;
      let upper_bound_length = 20;


      if (this.bodies[0].center[1] > upper_bound_center[1] - upper_bound_width - 1){
        this.bodies[0].linear_velocity[1] *= -1;
      }

      let left_bound_center = [-16, -10, 0];
      let left_bound_width = 1;
      let left_bound_length = 10;


      if (this.bodies[0].center[0] < left_bound_center[0] + left_bound_width + 1){
        this.bodies[0].linear_velocity[0] *= -1;
      }

      let right_bound_center = [16, -10, 0];
      let right_bound_width = 1;
      let right_bound_length = 20;


      if (this.bodies[0].center[0] > right_bound_center[0] - right_bound_width - 1){
        this.bodies[0].linear_velocity[0] *= -1;
      }

      //

      //////////////////////////////////////////////////////////
      // boundaries [THE END]
      //////////////////////////////////////////////////////////

      //////////////////////////////////////////////////////////
      // rectangular shapes, black ones
      //////////////////////////////////////////////////////////

      //right rectangular
      let rotation_radian = Math.PI/4;
      let rectangular_translation = [9, 2, 0];
      let rectangular_center_translated = Vec.of(9,2,0,0);
      let rectangular_center = Mat4.rotation(rotation_radian, Vec.of(0,0,1)).times(Mat4.translation(rectangular_translation)).times(rectangular_center_translated);
      rectangular_center[1] -= 10; // original
      let rectangular_width = 1;
      let rectangular_length = 5;

      let vertical_displacement = (1 + rectangular_width) * Math.sqrt(2);
      let horizontal_displacement = (1 + rectangular_length) * Math.sqrt(2);

      let slope = Math.sin(rotation_radian) / Math.cos(rotation_radian);
      let intercept = rectangular_center[1] - rectangular_center[0] * slope;
      let upper_intercept = intercept + vertical_displacement;
      let lower_intercept = intercept - vertical_displacement;
      let intercept_2 = rectangular_center[1] + rectangular_center[0] * slope;
      let slope_2 = -slope // TODO: just for 45 degree case
      let left_intercept = intercept_2 - horizontal_displacement;
      let right_intercept = intercept_2 + horizontal_displacement;

      if (this.collision_right_rectangle == true) {
        if (this.collision_right_rectangle_iterations == 6) {
          this.collision_right_rectangle = false;
          this.collision_right_rectangle_iterations = 0;
        }
        else
          this.collision_right_rectangle_iterations += 1;
      }

      // without the curvy edge implementation
      if (this.bodies[0].center[0] * slope + upper_intercept >= this.bodies[0].center[1] &&
          this.bodies[0].center[0] * slope + lower_intercept <= this.bodies[0].center[1] &&
          this.bodies[0].center[0] * slope_2 + left_intercept <= this.bodies[0].center[1] &&
          this.bodies[0].center[0] * slope_2 + right_intercept >= this.bodies[0].center[1] &&
          this.collision_right_rectangle == false)
          {
            //console.log(this.bodies[0].center[0])
            //console.log(this.bodies[0].center[1])
            let flag = true;

            if (Math.abs(this.bodies[0].center[0] * slope + upper_intercept - this.bodies[0].center[1]) < 0.5){

              let normal_vec_rect = Vec.of(-1,1,0).normalized(); // hard coded
              // same as circular shapes
              let cos_theta = HelperMethods.dot( this.bodies[0].linear_velocity, normal_vec_rect) / (HelperMethods.length(normal_vec_rect) * HelperMethods.length(this.bodies[0].linear_velocity));
              let theta = Math.acos(cos_theta);
              this.bodies[0].linear_velocity = this.bodies[0].linear_velocity.plus(normal_vec_rect.times((HelperMethods.length(this.bodies[0].linear_velocity) * cos_theta) * (-1.7)  ));
            }

            else if (Math.abs(this.bodies[0].center[0] * slope + lower_intercept - this.bodies[0].center[1]) < 0.5){

              let normal_vec_rect = Vec.of(1,-1,0).normalized(); // hard coded
              // same as circular shapes
              let cos_theta = HelperMethods.dot( this.bodies[0].linear_velocity, normal_vec_rect) / (HelperMethods.length(normal_vec_rect) * HelperMethods.length(this.bodies[0].linear_velocity));
              let theta = Math.acos(cos_theta);
              this.bodies[0].linear_velocity = this.bodies[0].linear_velocity.plus(normal_vec_rect.times((HelperMethods.length(this.bodies[0].linear_velocity) * cos_theta) * (-1.7)  ));
            }
            else if (Math.abs(this.bodies[0].center[0] * slope_2 + left_intercept - this.bodies[0].center[1]) < 0.5){

              let normal_vec_rect = Vec.of(-1,-1,0).normalized(); // hard coded
              // same as circular shapes
              let cos_theta = HelperMethods.dot( this.bodies[0].linear_velocity, normal_vec_rect) / (HelperMethods.length(normal_vec_rect) * HelperMethods.length(this.bodies[0].linear_velocity));
              let theta = Math.acos(cos_theta);
              this.bodies[0].linear_velocity = this.bodies[0].linear_velocity.plus(normal_vec_rect.times((HelperMethods.length(this.bodies[0].linear_velocity) * cos_theta) * (-1.7)  ));

            }
            else if (Math.abs(this.bodies[0].center[0] * slope_2 + right_intercept - this.bodies[0].center[1]) < 0.5){
              let normal_vec_rect = Vec.of(1,1,0).normalized(); // hard coded
              // same as circular shapes
              let cos_theta = HelperMethods.dot( this.bodies[0].linear_velocity, normal_vec_rect) / (HelperMethods.length(normal_vec_rect) * HelperMethods.length(this.bodies[0].linear_velocity));
              let theta = Math.acos(cos_theta);
              this.bodies[0].linear_velocity = this.bodies[0].linear_velocity.plus(normal_vec_rect.times((HelperMethods.length(this.bodies[0].linear_velocity) * cos_theta) * (-1.7)  ));
            }
            this.collision_right_rectangle = true;
          }


      // left rectangular
      let rotation_radian_l = Math.PI * 0.75;
      let rectangular_translation_l = [5, 2, 0];
      let rectangular_center_translated_l = Vec.of(5,2,0,0);
      let rectangular_center_l = Mat4.rotation(rotation_radian_l, Vec.of(0,0,1)).times(Mat4.translation(rectangular_translation_l)).times(rectangular_center_translated_l);
      rectangular_center_l[1] -= 10; // original
      let rectangular_width_l = 1;
      let rectangular_length_l = 5;


       let vertical_displacement_l = (1 + rectangular_width_l) * Math.sqrt(2);
       let horizontal_displacement_l = (1 + rectangular_length_l) * Math.sqrt(2);

       let slope_l = Math.sin(rotation_radian_l) / Math.cos(rotation_radian_l);
       let intercept_l = rectangular_center_l[1] - rectangular_center_l[0] * slope_l;
       let upper_intercept_l = intercept_l + vertical_displacement_l;
       let lower_intercept_l = intercept_l - vertical_displacement_l;
       let intercept_l_2 = rectangular_center_l[1] + rectangular_center_l[0] * slope_l;
       let slope_l_2 = -slope_l // TODO: just for 45 degree case
       let left_intercept_l = intercept_l_2 - horizontal_displacement_l;
       let right_intercept_l = intercept_l_2 + horizontal_displacement_l;

       if (this.collision_left_rectangle == true) {
        if (this.collision_left_rectangle_iterations == 6) {
          this.collision_left_rectangle = false;
          this.collision_left_rectangle_iterations = 0;
        }
        else
          this.collision_left_rectangle_iterations += 1;
      }

      // without the curvy edge implementation
       if (this.bodies[0].center[0] * slope_l + upper_intercept_l >= this.bodies[0].center[1] &&
           this.bodies[0].center[0] * slope_l + lower_intercept_l <= this.bodies[0].center[1] &&
           this.bodies[0].center[0] * slope_l_2 + left_intercept_l <= this.bodies[0].center[1] &&
           this.bodies[0].center[0] * slope_l_2 + right_intercept_l >= this.bodies[0].center[1] &&
           this.collision_left_rectangle == false)
           {

             //console.log(this.bodies[0].center[0])
             //console.log(this.bodies[0].center[1])

             if (Math.abs(this.bodies[0].center[0] * slope_l + upper_intercept_l - this.bodies[0].center[1]) < 0.5){

               let normal_vec_rect = Vec.of(1,1,0).normalized(); // hard coded
               // same as circular shapes
               let cos_theta = HelperMethods.dot( this.bodies[0].linear_velocity, normal_vec_rect) / (HelperMethods.length(normal_vec_rect) * HelperMethods.length(this.bodies[0].linear_velocity));
               let theta = Math.acos(cos_theta);
               this.bodies[0].linear_velocity = this.bodies[0].linear_velocity.plus(normal_vec_rect.times((HelperMethods.length(this.bodies[0].linear_velocity) * cos_theta) * (-1.7)  ));
               this.m_score += 20;
             }

             else if (Math.abs(this.bodies[0].center[0] * slope_l + lower_intercept_l - this.bodies[0].center[1]) < 0.5){

               let normal_vec_rect = Vec.of(-1,-1,0).normalized(); // hard coded
               // same as circular shapes
               let cos_theta = HelperMethods.dot( this.bodies[0].linear_velocity, normal_vec_rect) / (HelperMethods.length(normal_vec_rect) * HelperMethods.length(this.bodies[0].linear_velocity));
               let theta = Math.acos(cos_theta);
               this.bodies[0].linear_velocity = this.bodies[0].linear_velocity.plus(normal_vec_rect.times((HelperMethods.length(this.bodies[0].linear_velocity) * cos_theta) * (-1.7)  ));
               this.m_score += 20;

             }
             else if (Math.abs(this.bodies[0].center[0] * slope_l_2 + left_intercept_l - this.bodies[0].center[1]) < 0.5){

               let normal_vec_rect = Vec.of(1,-1,0).normalized(); // hard coded
               // same as circular shapes
               let cos_theta = HelperMethods.dot( this.bodies[0].linear_velocity, normal_vec_rect) / (HelperMethods.length(normal_vec_rect) * HelperMethods.length(this.bodies[0].linear_velocity));
               let theta = Math.acos(cos_theta);
               this.bodies[0].linear_velocity = this.bodies[0].linear_velocity.plus(normal_vec_rect.times((HelperMethods.length(this.bodies[0].linear_velocity) * cos_theta) * (-1.7)  ));
               this.m_score += 20;
             }
             else if (Math.abs(this.bodies[0].center[0] * slope_l_2 + right_intercept_l - this.bodies[0].center[1]) < 0.5){
               let normal_vec_rect = Vec.of(-1,1,0).normalized(); // hard coded
               // same as circular shapes
               let cos_theta = HelperMethods.dot( this.bodies[0].linear_velocity, normal_vec_rect) / (HelperMethods.length(normal_vec_rect) * HelperMethods.length(this.bodies[0].linear_velocity));
               let theta = Math.acos(cos_theta);
               this.bodies[0].linear_velocity = this.bodies[0].linear_velocity.plus(normal_vec_rect.times((HelperMethods.length(this.bodies[0].linear_velocity) * cos_theta) * (-1.7)  ));
               this.m_score += 20;
             }
             this.collision_left_rectangle = true;
           }

      //////////////////////////////////////////////////////////
      // rectangular shapes [THE END]
      //////////////////////////////////////////////////////////




      //////////////////////////////////////////////////////////
      // flippers
      //////////////////////////////////////////////////////////
      // left rectangular
      let left_rotation = this.left_flipper_rotation;
      let left_center = this.left_flipper_center
      let left_width = 1;
      let left_length = 4;


       let left_vertical_displacement = (1 + left_width)/Math.cos(left_rotation)
       let left_horizontal_displacement = (1 + left_length)/Math.sin(left_rotation);

       this.left_slope = Math.sin(left_rotation) / Math.cos(left_rotation);
       let left_interceptl = left_center[1] - left_center[0] * this.left_slope;
       this.left_upper_intercept = left_interceptl + left_vertical_displacement;
       this.left_lower_intercept = left_interceptl - left_vertical_displacement;
       this.left_slope_2 = -1/this.left_slope
       let left_intercept_2 = left_center[1] - left_center[0] * this.left_slope_2;
       this.left_left_intercept = left_intercept_2 - left_horizontal_displacement;
       this.left_right_intercept = left_intercept_2 + left_horizontal_displacement;

       if (this.collision_left_rectangle_flipper == true) {
        if (this.collision_left_rectangle_flipper_iterations == 6) {
          this.collision_left_rectangle_flipper = false;
          this.collision_left_rectangle_flipper_iterations = 0;
        }
        else
          this.collision_left_rectangle_flipper_iterations += 1;
      }

      // without the curvy edge implementation
       if (this.bodies[0].center[0] * this.left_slope + this.left_upper_intercept >= this.bodies[0].center[1] &&
           this.bodies[0].center[0] * this.left_slope + this.left_lower_intercept <= this.bodies[0].center[1] &&
           this.bodies[0].center[0] * this.left_slope_2 + this.left_left_intercept <= this.bodies[0].center[1] &&
           this.bodies[0].center[0] * this.left_slope_2 + this.left_right_intercept >= this.bodies[0].center[1] &&
           this.collision_left_rectangle_flipper == false)
           {
             //console.log(this.bodies[0].center[0])
             //console.log(this.bodies[0].center[1])

             if (Math.abs(this.bodies[0].center[0] * this.left_slope + this.left_upper_intercept - this.bodies[0].center[1]) < 0.5){

               let normal_vec_rect = Vec.of(-Math.sin(left_rotation),Math.cos(left_rotation),0).normalized(); // hard coded
               // same as circular shapes
               let cos_theta = HelperMethods.dot( this.bodies[0].linear_velocity, normal_vec_rect) / (HelperMethods.length(normal_vec_rect) * HelperMethods.length(this.bodies[0].linear_velocity));
               let theta = Math.acos(cos_theta);
               if (this.left_flag == false)
                 this.bodies[0].linear_velocity = this.bodies[0].linear_velocity.plus(normal_vec_rect.times((HelperMethods.length(this.bodies[0].linear_velocity) * cos_theta) * (-1.7)  ));
               else
                 this.bodies[0].linear_velocity = this.bodies[0].linear_velocity.plus(normal_vec_rect.times((HelperMethods.length(this.bodies[0].linear_velocity) * cos_theta) * (-3.5)  ));

             }

             else if (Math.abs(this.bodies[0].center[0] * this.left_slope + this.left_lower_intercept - this.bodies[0].center[1]) < 0.5){

               let normal_vec_rect = Vec.of(Math.sin(left_rotation),-Math.cos(left_rotation),0).normalized(); // hard coded
               // same as circular shapes
               let cos_theta = HelperMethods.dot( this.bodies[0].linear_velocity, normal_vec_rect) / (HelperMethods.length(normal_vec_rect) * HelperMethods.length(this.bodies[0].linear_velocity));
               let theta = Math.acos(cos_theta);
               if (this.left_flag == false)
                 this.bodies[0].linear_velocity = this.bodies[0].linear_velocity.plus(normal_vec_rect.times((HelperMethods.length(this.bodies[0].linear_velocity) * cos_theta) * (-1.7)  ));
               else
                 this.bodies[0].linear_velocity = this.bodies[0].linear_velocity.plus(normal_vec_rect.times((HelperMethods.length(this.bodies[0].linear_velocity) * cos_theta) * (-3.5)  ));
             }
             else if (Math.abs(this.bodies[0].center[0] * this.left_slope_2 + this.left_left_intercept - this.bodies[0].center[1]) < 0.5){

               let normal_vec_rect = Vec.of(-Math.cos(left_rotation),-Math.sin(left_rotation),0).normalized(); // hard coded
               // same as circular shapes
               let cos_theta = HelperMethods.dot( this.bodies[0].linear_velocity, normal_vec_rect) / (HelperMethods.length(normal_vec_rect) * HelperMethods.length(this.bodies[0].linear_velocity));
               let theta = Math.acos(cos_theta);
               if (this.left_flag == false)
                 this.bodies[0].linear_velocity = this.bodies[0].linear_velocity.plus(normal_vec_rect.times((HelperMethods.length(this.bodies[0].linear_velocity) * cos_theta) * (-1.7)  ));
               else
                 this.bodies[0].linear_velocity = this.bodies[0].linear_velocity.plus(normal_vec_rect.times((HelperMethods.length(this.bodies[0].linear_velocity) * cos_theta) * (-3.5)  ));

             }
             else if (Math.abs(this.bodies[0].center[0] * this.left_slope_2 + this.left_right_intercept - this.bodies[0].center[1]) < 0.5){
               let normal_vec_rect = Vec.of(Math.cos(left_rotation),Math.sin(left_rotation),0).normalized(); // hard coded
               // same as circular shapes
               let cos_theta = HelperMethods.dot( this.bodies[0].linear_velocity, normal_vec_rect) / (HelperMethods.length(normal_vec_rect) * HelperMethods.length(this.bodies[0].linear_velocity));
               let theta = Math.acos(cos_theta);
               if (this.left_flag == false)
                 this.bodies[0].linear_velocity = this.bodies[0].linear_velocity.plus(normal_vec_rect.times((HelperMethods.length(this.bodies[0].linear_velocity) * cos_theta) * (-1.7)  ));
               else
                 this.bodies[0].linear_velocity = this.bodies[0].linear_velocity.plus(normal_vec_rect.times((HelperMethods.length(this.bodies[0].linear_velocity) * cos_theta) * (-3.5)  ));
             }
             this.collision_left_rectangle_flipper = true;
           }

     //right rectangular flipper


      let right_rotation = this.right_flipper_rotation;
      let right_center = this.right_flipper_center;
      let right_width = 1;
      let right_length = 4;

       let right_vertical_displacement = (1 + right_width)/Math.cos(Math.PI - right_rotation)
       let right_horizontal_displacement = (1 + right_length)/Math.sin(Math.PI - right_rotation);

       this.right_slope = Math.sin(right_rotation) / Math.cos(right_rotation);
       //console.log(this.right_slope)
       let right_interceptl = right_center[1] - right_center[0] * this.right_slope;
       this.right_upper_intercept = right_interceptl + right_vertical_displacement;
       this.right_lower_intercept = right_interceptl - right_vertical_displacement;
       this.right_slope_2 = -1/this.right_slope
       let right_intercept_2 = right_center[1] - right_center[0] * this.right_slope_2;

       this.right_left_intercept = right_intercept_2 - right_horizontal_displacement;
       this.right_right_intercept = right_intercept_2 + right_horizontal_displacement;

       if (this.collision_right_rectangle_flipper == true) {
        if (this.collision_right_rectangle_flipper_iterations == 6) {
          this.collision_right_rectangle_flipper = false;
          this.collision_right_rectangle_flipper_iterations = 0;
        }
        else
          this.collision_right_rectangle_flipper_iterations += 1;
      }

      // without the curvy edge implementation
       if (this.bodies[0].center[0] * this.right_slope + this.right_upper_intercept >= this.bodies[0].center[1] &&
           this.bodies[0].center[0] * this.right_slope + this.right_lower_intercept <= this.bodies[0].center[1] &&
           this.bodies[0].center[0] * this.right_slope_2 + this.right_left_intercept <= this.bodies[0].center[1] &&
           this.bodies[0].center[0] * this.right_slope_2 + this.right_right_intercept >= this.bodies[0].center[1] &&
           this.collision_right_rectangle_flipper == false)
           {
             //console.log(this.bodies[0].center[0])
             //console.log(this.bodies[0].center[1])

             if (Math.abs(this.bodies[0].center[0] * this.right_slope + this.right_upper_intercept - this.bodies[0].center[1]) < 0.5){

               let normal_vec_rect = Vec.of(-Math.sin(right_rotation),Math.cos(right_rotation),0).normalized(); // hard coded
               // same as circular shapes
               let cos_theta = HelperMethods.dot( this.bodies[0].linear_velocity, normal_vec_rect) / (HelperMethods.length(normal_vec_rect) * HelperMethods.length(this.bodies[0].linear_velocity));
               let theta = Math.acos(cos_theta);
               if (this.right_flag == false)
                 this.bodies[0].linear_velocity = this.bodies[0].linear_velocity.plus(normal_vec_rect.times((HelperMethods.length(this.bodies[0].linear_velocity) * cos_theta) * (-1.7)  ));
               else
                 this.bodies[0].linear_velocity = this.bodies[0].linear_velocity.plus(normal_vec_rect.times((HelperMethods.length(this.bodies[0].linear_velocity) * cos_theta) * (-3.5)  ));

             }

             else if (Math.abs(this.bodies[0].center[0] * this.right_slope + this.right_lower_intercept - this.bodies[0].center[1]) < 0.5){

               let normal_vec_rect = Vec.of(Math.sin(right_rotation),-Math.cos(right_rotation),0).normalized(); // hard coded
               // same as circular shapes
               let cos_theta = HelperMethods.dot( this.bodies[0].linear_velocity, normal_vec_rect) / (HelperMethods.length(normal_vec_rect) * HelperMethods.length(this.bodies[0].linear_velocity));
               let theta = Math.acos(cos_theta);
               if (this.right_flag == false)
                 this.bodies[0].linear_velocity = this.bodies[0].linear_velocity.plus(normal_vec_rect.times((HelperMethods.length(this.bodies[0].linear_velocity) * cos_theta) * (-1.7)  ));
               else
                 this.bodies[0].linear_velocity = this.bodies[0].linear_velocity.plus(normal_vec_rect.times((HelperMethods.length(this.bodies[0].linear_velocity) * cos_theta) * (-3.5)  ));
             }
             else if (Math.abs(this.bodies[0].center[0] * this.right_slope_2 + this.right_left_intercept - this.bodies[0].center[1]) < 0.5){

               let normal_vec_rect = Vec.of(-Math.cos(right_rotation),-Math.sin(right_rotation),0).normalized(); // hard coded
               // same as circular shapes
               let cos_theta = HelperMethods.dot( this.bodies[0].linear_velocity, normal_vec_rect) / (HelperMethods.length(normal_vec_rect) * HelperMethods.length(this.bodies[0].linear_velocity));
               let theta = Math.acos(cos_theta);
               if (this.right_flag == false)
                 this.bodies[0].linear_velocity = this.bodies[0].linear_velocity.plus(normal_vec_rect.times((HelperMethods.length(this.bodies[0].linear_velocity) * cos_theta) * (-1.7)  ));
               else
                 this.bodies[0].linear_velocity = this.bodies[0].linear_velocity.plus(normal_vec_rect.times((HelperMethods.length(this.bodies[0].linear_velocity) * cos_theta) * (-3.5)  ));

             }
             else if (Math.abs(this.bodies[0].center[0] * this.right_slope_2 + this.right_right_intercept - this.bodies[0].center[1]) < 0.5){
               let normal_vec_rect = Vec.of(Math.cos(right_rotation),Math.sin(right_rotation),0).normalized(); // hard coded
               // same as circular shapes
               let cos_theta = HelperMethods.dot( this.bodies[0].linear_velocity, normal_vec_rect) / (HelperMethods.length(normal_vec_rect) * HelperMethods.length(this.bodies[0].linear_velocity));
               let theta = Math.acos(cos_theta);
               if (this.right_flag == false)
                 this.bodies[0].linear_velocity = this.bodies[0].linear_velocity.plus(normal_vec_rect.times((HelperMethods.length(this.bodies[0].linear_velocity) * cos_theta) * (-1.7)  ));
               else
                 this.bodies[0].linear_velocity = this.bodies[0].linear_velocity.plus(normal_vec_rect.times((HelperMethods.length(this.bodies[0].linear_velocity) * cos_theta) * (-3.5)  ));
             }
             this.collision_right_rectangle_flipper = true;
           }

       let j = 0;

       while (j < 2) {
         if (Math.abs(this.bodies[0].linear_velocity[j]) > 8) {
           if (this.bodies[0].linear_velocity[j] > 0)
             this.bodies[0].linear_velocity[j] = 8;
           else
             this.bodies[0].linear_velocity[j] = -8;
         }
         j += 1;
       }



      //////////////////////////////////////////////////////////
      // flippers [THE END]
      //////////////////////////////////////////////////////////

      // Black Hole Teleportation
      if ((this.bodies[0].center[0] - 7)**2 + (this.bodies[0].center[1] + 8)**2 < this.sun_size + 1){
          this.bodies[0].center[0] = -5;
          this.bodies[0].center[1] = 5;
          this.bodies[0].linear_velocity = Vec.of(0,-1,0).randomized(2).normalized().times(3);
          this.bodies[0].linear_velocity[2] = 0
      }
                                                      // Delete bodies that stop or stray too far away:


      if(this.bodies.length > 0 && this.bodies[0].center[1]<= -30){
        this.m_lives -= 1;
        this.steps_taken = 0;
        this.bodies.pop();
      }
       //this.bodies = this.bodies.filter( b => b.center.norm() < 50 && b.linear_velocity.norm() > 2 );
    }
  display( context, program_state )
    {                                 // display(): Draw everything else in the scene besides the moving bodies.
      program_state.score = this.m_score;
      program_state.lives = this.m_lives;
      super.display( context, program_state );

      if( !context.scratchpad.controls )
        { this.children.push( context.scratchpad.controls = new defs.Movement_Controls() );
          this.children.push( new defs.Program_State_Viewer() );
          program_state.set_camera( Mat4.look_at( Vec.of( 0,-40,40 ), Vec.of( 0,-15,0 ), Vec.of( 0,0,100 ) ));    // Locate the camera here (inverted matrix).
          program_state.projection_transform = Mat4.perspective( Math.PI/4, context.width/context.height, 1, 500 );
        }
      program_state.lights = [ new Light( Vec.of( -10,-20, 30,1 ), Color.of( 1,1,1,1 ), 2000 ) ];
                                                                                              // Draw the ground:

      let model_transform = Mat4.identity();
      model_transform = model_transform.times(Mat4.translation([ 0,-10,0 ]));

      let flip_time = this.steps_taken;
//       console.log(flip_time);

      // obstacles
      this.shapes.capped.draw( context, program_state, model_transform.times(Mat4.translation([-3, 12,.5])), this.material.cylinder1);
      this.shapes.capped.draw( context, program_state, model_transform.times(Mat4.translation([3, 12,.5])), this.material.cylinder2);
      this.shapes.capped.draw( context, program_state, model_transform.times(Mat4.translation([0, 15,.5])), this.material.cylinder3);
      this.shapes.square.draw( context, program_state, model_transform.times( Mat4.scale([ 15,20,1 ]) ), this.material.background);
      this.shapes.cube.draw( context, program_state, model_transform.times(Mat4.rotation(Math.PI/4, Vec.of(0,0,1))).times(Mat4.translation([9,2,1])).times(Mat4.scale([5,1,1])), this.material.blocks );
      this.shapes.cube.draw( context, program_state, model_transform.times(Mat4.rotation(Math.PI*0.75, Vec.of(0,0,1))).times(Mat4.translation([5,2,1])).times(Mat4.scale([5,1,1])), this.material.blocks );
//       this.shapes.arc.draw( context, program_state, model_transform.times(Mat4.translation([0,-20,1])).times(Mat4.rotation( Math.PI/4, Vec.of( 0,0,1 ))).times(Mat4.scale([10,10,2])), this.material.donuts);
//       this.shapes.arc.draw( context, program_state, model_transform.times(Mat4.translation([0,5,1])).times(Mat4.rotation( -Math.PI/2, Vec.of( 0,0,1 ))).times(Mat4.scale([10,10,2])), this.material.donuts);
      if(this.steps_taken<130)
      {
        this.shapes.arc.draw( context, program_state, model_transform.times(Mat4.translation([8,13,1])).times(Mat4.rotation( 0, Vec.of( 0,0,1 ))).times(Mat4.scale([7,7,1.5])), this.material.donuts);
      this.shapes.arc.draw( context, program_state, model_transform.times(Mat4.translation([6,11,1])).times(Mat4.rotation( 0, Vec.of( 0,0,1 ))).times(Mat4.scale([6.5,6.5,1.5])), this.material.donuts);
      }
      this.shapes.arc.draw( context, program_state, model_transform.times(Mat4.translation([6.2,-6,1])).times(Mat4.rotation( -Math.PI/2.5, Vec.of( 0,0,1 ))).times(Mat4.scale([6.3,6.3,1.5])), this.material.donuts);
      this.shapes.arc.draw( context, program_state, model_transform.times(Mat4.translation([-6.2,-6,1])).times(Mat4.rotation( Math.PI/2+Math.PI/2.5, Vec.of( 0,0,1 ))).times(Mat4.scale([6.3,6.3,1.5])), this.material.donuts);

      //console.log(model_transform.times(Vec.of(0,0,0,1)))



      if (this.left_flag == false){
        this.shapes.cube.draw(context, program_state, model_transform.times(Mat4.rotation(Math.PI/4, Vec.of(0,0,1))).times(Mat4.translation([-7,-15,1])).times(Mat4.scale([4,1,1])), this.material.levers );
        let rectangular_translation_ll = [-7,-15, 0];
        let rectangular_center_translated_ll = Vec.of(-7,-15,0,0);
        this.left_flipper_rotation = Math.PI/4;
        this.left_flipper_center = Mat4.rotation(this.left_flipper_rotation, Vec.of(0,0,1)).times(Mat4.translation(rectangular_translation_ll)).times(rectangular_center_translated_ll);
        this.left_flipper_center[1] -= 10;
      }
      else {

        let flipper_rotation = -0.3 - 0.3 * Math.sin((flip_time - this.left_value)/5 - Math.PI/2);
        this.shapes.cube.draw(context, program_state, model_transform.times(Mat4.rotation(Math.PI/4, Vec.of(0,0,1))).times(Mat4.translation([-3,-15,1])).times(Mat4.rotation(flipper_rotation, Vec.of(0,0,1))).times(Mat4.translation([-4, 0, 0])).times(Mat4.scale([4,1,1])), this.material.levers );

        if (flip_time - this.left_value > 314 / 10){
          this.left_flag=false;
          this.left_value =0;
        }
        this.left_flipper_rotation = Math.PI/4 + flipper_rotation;
        this.left_flipper_center = (model_transform.times(Mat4.rotation(Math.PI/4, Vec.of(0,0,1))).times(Mat4.translation([-3,-15,1])).times(Mat4.rotation(flipper_rotation, Vec.of(0,0,1))).times(Mat4.translation([-4, 0, 0])).times(Vec.of(0,0,0,1))).to3();
        this.left_flipper_center[2] = 0;
        //console.log(this.left_flipper_rotation)
      }

      if (this.right_flag == false) {
        this.shapes.cube.draw( context, program_state, model_transform.times(Mat4.rotation(Math.PI*0.75, Vec.of(0,0,1))).times(Mat4.translation([-7,15,1])).times(Mat4.scale([4,1,1])), this.material.levers );
        let rectangular_translation_r = [-7, 15, 0];
        let rectangular_center_translated_r = Vec.of(-7, 15, 0, 0);
        this.right_flipper_rotation = Math.PI*0.75;
        this.right_flipper_center = Mat4.rotation(this.right_flipper_rotation, Vec.of(0,0,1)).times(Mat4.translation(rectangular_translation_r)).times(rectangular_center_translated_r);
        this.right_flipper_center[1] -= 10;
        //console.log(this.right_flipper_center)
      }
      else {
        let flipper_rotation = 0.3 + 0.3 * Math.sin((flip_time - this.right_value)/5 - Math.PI/2);
        this.shapes.cube.draw(context, program_state, model_transform.times(Mat4.rotation(Math.PI*0.75, Vec.of(0,0,1))).times(Mat4.translation([-3,15,1])).times(Mat4.rotation(flipper_rotation, Vec.of(0,0,1))).times(Mat4.translation([-4, 0, 0])).times(Mat4.scale([4,1,1])), this.material.levers );
        if (flip_time - this.right_value > 314 / 10 ){
          this.right_flag=false;
          this.right_value =0;
        }
        this.right_flipper_rotation = Math.PI*0.75 + flipper_rotation;
        this.right_flipper_center = (model_transform.times(Mat4.rotation(Math.PI*0.75, Vec.of(0,0,1))).times(Mat4.translation([-3,15,1])).times(Mat4.rotation(flipper_rotation, Vec.of(0,0,1))).times(Mat4.translation([-4, 0, 0]))).times(Vec.of(0,0,0,1)).to3();
        this.right_flipper_center[2] = 0;
      }



     //Black hole effects:

       const t = program_state.animation_time / 200;
       const smoothly_varying_ratio = 1 + .5 * Math.sin( 2 * Math.PI * t/10 );
        this.sun_size = 1 + 2 * smoothly_varying_ratio;
       const sun = Mat4.scale([this.sun_size, this.sun_size,this.sun_size]);
       //sun_color = yellow.times(smoothly_varying_ratio).plus(blue.times((1 - smoothly_varying_ratio)));


     this.shapes.ball_6.draw( context, program_state, model_transform.times(Mat4.translation([7, 2, 1])).times(sun), this.material.black_hole)

     this.shapes.ball_6.draw( context, program_state, model_transform.times(Mat4.translation([-5, 15, 1])), this.material.black_hole)





     //Black hole effects [END]



      // maybe randomize the cone positions
      //this.shapes.cone.draw( context, program_state, model_transform.times(Mat4.translation([0,10,3])), this.material.cylinder);
//       this.shapes.cone.draw( context, program_state, model_transform.times(Mat4.translation([this.right_flipper_center[0], this.right_flipper_center[1] + 10,3])), this.material.cylinder1);
//       this.shapes.cone.draw( context, program_state, model_transform.times(Mat4.translation([this.right_flipper_center[0], this.right_flipper_center[0] * this.right_slope + this.right_upper_intercept + 10,3])), this.material.cylinder1);
//       this.shapes.cone.draw( context, program_state, model_transform.times(Mat4.translation([this.right_flipper_center[0], this.right_flipper_center[0] * this.right_slope + this.right_lower_intercept + 10,3])), this.material.cylinder1);
//       this.shapes.cone.draw( context, program_state, model_transform.times(Mat4.translation([this.right_flipper_center[0], this.right_flipper_center[0] * this.right_slope_2 + this.right_left_intercept + 10,3])), this.material.cylinder1);
//       this.shapes.cone.draw( context, program_state, model_transform.times(Mat4.translation([this.right_flipper_center[0], this.right_flipper_center[0] * this.right_slope_2 + this.right_right_intercept + 10,3])), this.material.cylinder1);
//       // obstacles [END]

      // boundaries
      this.shapes.cube.draw( context, program_state, model_transform.times(Mat4.translation([0, 21, 1])).times(Mat4.scale([15,1,1])), this.material.background)
      this.shapes.cube.draw( context, program_state, model_transform.times(Mat4.translation([16, 0, 1])).times(Mat4.scale([1,20,1])), this.material.background)
      this.shapes.cube.draw( context, program_state, model_transform.times(Mat4.translation([-16, 0, 1])).times(Mat4.scale([1,20,1])), this.material.background)
      if(this.steps_taken<130){
      this.shapes.cube.draw( context, program_state, model_transform.times(Mat4.translation([12.5, -4, 1])).times(Mat4.scale([0.2,16,0.5])), this.material.donuts)}
      // boundaries [END]

    }
}


const Sun_Shader = defs.Sun_Shader =
class Sun_Shader extends Shader
{ update_GPU( context, gpu_addresses, program_state, model_transform, material )
    {
                      // TODO (#EC 2): Pass the same information to the shader as for EC part 1.  Additionally
                      // pass material.color to the shader.
        const [ P, C, M ] = [ program_state.projection_transform, program_state.camera_inverse, model_transform ],
                      PCM = P.times( C ).times( M );
        context.uniformMatrix4fv( gpu_addresses.projection_camera_model_transform, false, Mat.flatten_2D_to_1D( PCM.transposed() ) );
        context.uniform1f ( gpu_addresses.animation_time, program_state.animation_time / 1000 );
        context.uniform4fv( gpu_addresses.sun_color, material.color);
    }
                                // TODO (#EC 2):  Complete the shaders, displacing the input sphere's vertices as
                                // a fireball effect and coloring fragments according to displacement.

  shared_glsl_code()            // ********* SHARED CODE, INCLUDED IN BOTH SHADERS *********
    { return `precision mediump float;
              varying float disp;

      `;
    }
  vertex_glsl_code()           // ********* VERTEX SHADER *********
    { return this.shared_glsl_code() + `

attribute vec3 position;
attribute vec3 normal;

varying float noise;
uniform float animation_time;
uniform mat4 projection_camera_model_transform;
const float pulseHeight = 0.02;
const float displacementHeight = 0.9;
const float turbulenceDetail = 0.5;

vec3 mod289(vec3 x) {
  return x - floor(x * (1.0 / 289.0)) * 289.0;
}

vec4 mod289(vec4 x) {
  return x - floor(x * (1.0 / 289.0)) * 289.0;
}

vec4 permute(vec4 x) {
  return mod289(((x*34.0)+1.0)*x);
}

vec4 taylorInvSqrt(vec4 r) {
  return 1.79284291400159 - 0.85373472095314 * r;
}

vec3 fade(vec3 t) {
  return t*t*t*(t*(t*6.0-15.0)+10.0);
}

// Klassisk Perlin noise
float cnoise(vec3 P) {
  vec3 Pi0 = floor(P); // indexing
  vec3 Pi1 = Pi0 + vec3(1.0); // Integer part + 1
  Pi0 = mod289(Pi0);
  Pi1 = mod289(Pi1);
  vec3 Pf0 = fract(P); // Fractional part for interpolation
  vec3 Pf1 = Pf0 - vec3(1.0); // Fractional part - 1.0
  vec4 ix = vec4(Pi0.x, Pi1.x, Pi0.x, Pi1.x);
  vec4 iy = vec4(Pi0.yy, Pi1.yy);
  vec4 iz0 = Pi0.zzzz;
  vec4 iz1 = Pi1.zzzz;

  vec4 ixy = permute(permute(ix) + iy);
  vec4 ixy0 = permute(ixy + iz0);
  vec4 ixy1 = permute(ixy + iz1);

  vec4 gx0 = ixy0 * (1.0 / 7.0);
  vec4 gy0 = fract(floor(gx0) * (1.0 / 7.0)) - 0.5;
  gx0 = fract(gx0);
  vec4 gz0 = vec4(0.5) - abs(gx0) - abs(gy0);
  vec4 sz0 = step(gz0, vec4(0.0));
  gx0 -= sz0 * (step(0.0, gx0) - 0.5);
  gy0 -= sz0 * (step(0.0, gy0) - 0.5);

  vec4 gx1 = ixy1 * (1.0 / 7.0);
  vec4 gy1 = fract(floor(gx1) * (1.0 / 7.0)) - 0.5;
  gx1 = fract(gx1);
  vec4 gz1 = vec4(0.5) - abs(gx1) - abs(gy1);
  vec4 sz1 = step(gz1, vec4(0.0));
  gx1 -= sz1 * (step(0.0, gx1) - 0.5);
  gy1 -= sz1 * (step(0.0, gy1) - 0.5);

  vec3 g000 = vec3(gx0.x,gy0.x,gz0.x);
  vec3 g100 = vec3(gx0.y,gy0.y,gz0.y);
  vec3 g010 = vec3(gx0.z,gy0.z,gz0.z);
  vec3 g110 = vec3(gx0.w,gy0.w,gz0.w);
  vec3 g001 = vec3(gx1.x,gy1.x,gz1.x);
  vec3 g101 = vec3(gx1.y,gy1.y,gz1.y);
  vec3 g011 = vec3(gx1.z,gy1.z,gz1.z);
  vec3 g111 = vec3(gx1.w,gy1.w,gz1.w);

  vec4 norm0 = taylorInvSqrt(vec4(dot(g000, g000), dot(g010, g010), dot(g100, g100), dot(g110, g110)));
  g000 *= norm0.x;
  g010 *= norm0.y;
  g100 *= norm0.z;
  g110 *= norm0.w;
  vec4 norm1 = taylorInvSqrt(vec4(dot(g001, g001), dot(g011, g011), dot(g101, g101), dot(g111, g111)));
  g001 *= norm1.x;
  g011 *= norm1.y;
  g101 *= norm1.z;
  g111 *= norm1.w;

  float n000 = dot(g000, Pf0);
  float n100 = dot(g100, vec3(Pf1.x, Pf0.yz));
  float n010 = dot(g010, vec3(Pf0.x, Pf1.y, Pf0.z));
  float n110 = dot(g110, vec3(Pf1.xy, Pf0.z));
  float n001 = dot(g001, vec3(Pf0.xy, Pf1.z));
  float n101 = dot(g101, vec3(Pf1.x, Pf0.y, Pf1.z));
  float n011 = dot(g011, vec3(Pf0.x, Pf1.yz));
  float n111 = dot(g111, Pf1);

  vec3 fade_xyz = fade(Pf0);
  vec4 n_z = mix(vec4(n000, n100, n010, n110), vec4(n001, n101, n011, n111), fade_xyz.z);
  vec2 n_yz = mix(n_z.xy, n_z.zw, fade_xyz.y);
  float n_xyz = mix(n_yz.x, n_yz.y, fade_xyz.x);
  return 2.2 * n_xyz;
}

// Ashima code
float turbulence( vec3 p ) {
    float t = -0.5;
    for (float f = 1.0 ; f <= 10.0 ; f++ ){
        float power = pow( 2.0, f );
        t += abs( cnoise( vec3( power * p ) ) / power );
    }
    return t;
}

void main() {
    noise = -0.8 * turbulence( turbulenceDetail * normal + ( animation_time * 1.0 ) );

    float b = pulseHeight * cnoise(
        0.05 * position + vec3( 1.0 * animation_time )
    );
    float displacement = ( 0.0 - displacementHeight ) * noise + b;

    vec3 newPosition = position + normal * displacement;
    gl_Position = projection_camera_model_transform * vec4( newPosition, 1.0 );
    disp = 15.0 * displacement;
}`;
    }
  fragment_glsl_code()           // ********* FRAGMENT SHADER *********
    { return this.shared_glsl_code() + `
        uniform vec4 sun_color;
        void main()
        {

            vec3 color = vec3((1.-disp), (0.1-disp*0.2)+0.1, (0.1-disp*0.1)+0.1*abs(sin(disp)));
            gl_FragColor = vec4( color.rgb, 1.0 );
            gl_FragColor *= sun_color;

        } ` ;
    }
}
