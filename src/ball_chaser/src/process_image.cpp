#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Camera sensor Field of view (FOV) which's defined in gazebo plugin
#define HORIZONTAL_FOV 1.3962634
/*
 *  Helper functions for converting RGB to HSL color model
 *  HSL color is easier to identify white color if there is ambiance light involve
 *  class RGB is the object structure of RGB
 *  class HSL is the object structure of HSL
 *
 *  function RGBToHSL to convert RGB instant and then return HSL instant
 *  
 *  ref.from https://www.programmingalgorithms.com/algorithm/rgb-to-hsl/cpp/
 */

// RGB class for storing pixel in rgb format
class RGB
{
public:
  unsigned char R;
  unsigned char G;
  unsigned char B;

    RGB (unsigned char r, unsigned char g, unsigned char b)
  {
    R = r;
    G = g;
    B = b;
  }

  bool Equals (RGB rgb)
  {
    return (R == rgb.R) && (G == rgb.G) && (B == rgb.B);
  }
};

// HSL class for storing pixel in hsl format
class HSL
{
public:
  int H;
  float S;
  float L;

    HSL (int h, float s, float l)
  {
    H = h;
    S = s;
    L = l;
  }

  bool Equals (HSL hsl)
  {
    return (H == hsl.H) && (S == hsl.S) && (L == hsl.L);
  }
};


//RGBToHSL to convert RGB instant and then return HSL instant
static HSL RGBToHSL (RGB rgb)
{
  HSL hsl = HSL (0, 0, 0);

  float r = (rgb.R / 255.0f);
  float g = (rgb.G / 255.0f);
  float b = (rgb.B / 255.0f);

  float min_val = std::min (std::min (r, g), b);
  float max_val = std::max (std::max (r, g), b);
  float delta = max_val - min_val;

  hsl.L = (max_val + min_val) / 2;

  if (delta == 0)
    {
      hsl.H = 0;
      hsl.S = 0.0f;
    }
  else
    {
      hsl.S =
	(hsl.L <= 0.5) ? (delta / (max_val + min_val)) : (delta / (2 - max_val - min_val));

      float hue;

      if (r == max_val)
	{
	  hue = ((g - b) / 6) / delta;
	}
      else if (g == max_val)
	{
	  hue = (1.0f / 3) + ((b - r) / 6) / delta;
	}
      else
	{
	  hue = (2.0f / 3) + ((r - g) / 6) / delta;
	}

      if (hue < 0)
	hue += 1;
      if (hue > 1)
	hue -= 1;

      hsl.H = (int) (hue * 360);
    }

  return hsl;
}



/*
 * The map function is for changing one range of values into another range of values 
 * x is input value
 * in_min would get mapped to out_min
 * in_max to out_max
 */
float map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


/*
 * The find_white_pixel function is to convert RGB8 encoded image to HSL color
 * and create histogram of white pixel that can pass threshold into x_lightness[800] array
 */
int find_white_pixel (const sensor_msgs::Image img, float light_thres,
		  bool verbose)
{
  // Create array to keep histogram of white pixel
  int x_lightness[img.width] = { 0 };
  // Tmp val of RGB
  int r, g, b;
  // Tmp instant of HSL
  HSL hsl_value = HSL (0, 0, 0);

  // For 800 loop for y axis
  for (int i = 0; i < img.height; i++)
    // For 800 loop for x axis
    for (int j = 0, x = 0; j < img.step; j = j + 3, x++)
      {
        // Read 3 bytes of RGB8 data store in r, g, b
	r = img.data[i * img.step + j];
	g = img.data[i * img.step + j + 1];
	b = img.data[i * img.step + j + 2];
        // Create RGB instant and call function RGBToHSL then store in hsl_value
	hsl_value = RGBToHSL (RGB (r, g, b));

	// If the pixel has a higher L value than the threshold then
        // x_lightness plus 1 of this particular x position
	if (hsl_value.L >= light_thres)
	  {
	    ++x_lightness[x];

	  }
      }

  // Vistualize white pixel histogram
  if (verbose)
    {
      for (int i = 0; i < img.width; i++)
	{
	  // 1 score = 10 pixels
	  int score = x_lightness[i] / 10;
	  std::string o = "";
	  for (int j = 0; j < score; j++)
	    o = o + "o";
	  //ROS_INFO ("[%d]:%3d %s", i, x_lightness[i], o.c_str ());
	}
    }
  // Find the maximum value in x_lightness then its index  
  int x_of_white =
    std::distance (x_lightness,
		   std::max_element (x_lightness, x_lightness + img.width));
  
  // If no white pixel then function return -1
  if (x_lightness[x_of_white] == 0)
    {
      ROS_INFO ("White pixel's not found!");
      return -1;
    }
  // Return position x of the maximum white pixel
  ROS_INFO ("1st maximum white pixel at X: %d", x_of_white);
  return x_of_white;
}


// Define a global client that can request services
ros::ServiceClient client;


// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot (float lin_x, float ang_z)
{
  // TODO: Request a service and pass the velocities to it to drive the robot
  ball_chaser::DriveToTarget drive_cmd;
  drive_cmd.request.linear_x = lin_x;
  drive_cmd.request.angular_z = ang_z;

  if (!client.call (drive_cmd))
    {
      ROS_ERROR ("Failed to execute drive command");
    }

  // Request to drive
}





// This callback function continuously executes and reads the image data
void process_image_callback (const sensor_msgs::Image img)
{

  // TODO: Loop through each pixel in the image and check if there's a bright white one
  // Then, identify if this pixel falls in the left, mid, or right side of the image
  // Depending on the white ball position, call the drive_bot function and pass velocities to it
  // Request a stop when there's no white ball seen by the camera

  // Define white lightness (L) threshold of HSL, if the value is 1.0 means pure white
  // It can work more tolerance to ambiance light change by setting light_thres lower than 1.0 e.g. 0.95
  const float light_thres = 1.0;
  int x_of_white;
  float linear_x = 0.0, angular_z = 0.0,

  // Use x=0 in map function to find coff of camera sensor field of view vs -1 to 1
  angular_fov_coff = map (0, -1, 1, 0, HORIZONTAL_FOV);
  // Call find_white_pixel to locate the maximum white pixel in the x-axis
  x_of_white = find_white_pixel (img, light_thres, false);

  // If no white pixel's found then rotate around
  if (x_of_white < 0)
    {
      linear_x = 0.0;
      angular_z = -.5;
    }
  else
    {
      // Move forward
      linear_x = 0.1;
      // Mapping x position of the white pixel to angular_z
      // the result of angular_x will head robot about to the center of the white ball
      angular_z = -map (x_of_white, 0, img.width - 1, -angular_fov_coff,
			angular_fov_coff);
      ROS_INFO ("White at x :%d, calculated angular_z:%f", x_of_white, angular_z);
    }
  drive_robot (linear_x, angular_z);

}

int main (int argc, char **argv)
{
  // Initialize the process_image node and create a handle to it
  ros::init (argc, argv, "process_image");
  ros::NodeHandle n;

  // Define a client service capable of requesting services from command_robot
  client =
    n.serviceClient < ball_chaser::DriveToTarget >
    ("/ball_chaser/command_robot");

  // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
  ros::Subscriber sub1 =
    n.subscribe ("/camera/rgb/image_raw", 10, process_image_callback);

  // Handle ROS communication events
  ros::spin ();

  return 0;
}

