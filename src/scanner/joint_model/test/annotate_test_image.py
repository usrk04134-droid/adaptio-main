#!/usr/bin/env python3
"""
Tool to annotate test images with expected ABW points for BigSnake::Parse() tests.
This tool helps in creating test data by allowing manual annotation of ABW points
on test images and exports them in YAML format.
"""

import cv2
import numpy as np
import yaml
import argparse
import os
from typing import List, Tuple, Dict, Any

class ABWAnnotator:
    """Interactive tool for annotating ABW points on test images."""
    
    ABW_NAMES = [
        "ABW0 (Left top edge)",
        "ABW1 (Left wall upper)",
        "ABW2 (Left wall lower)",
        "ABW3 (Bottom left)",
        "ABW4 (Bottom center/root)",
        "ABW5 (Bottom right)",
        "ABW6 (Right wall lower)",
        "ABW7 (Right wall upper)",
        "ABW8 (Right top edge)",
        "ABW9 (Reserved)",
        "ABW10 (Reserved)",
        "ABW11 (Reserved)",
        "ABW12 (Reserved)",
        "ABW13 (Reserved)",
        "ABW14 (Reserved)"
    ]
    
    def __init__(self, image_path: str):
        self.image_path = image_path
        self.image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if self.image is None:
            raise ValueError(f"Failed to load image: {image_path}")
        
        self.display_image = cv2.cvtColor(self.image, cv2.COLOR_GRAY2BGR)
        self.abw_points: List[Tuple[float, float]] = []
        self.current_point_idx = 0
        self.scale = 1.0
        self.window_name = "ABW Point Annotator"
        
        # Image to workspace transformation (example values - adjust as needed)
        self.pixel_to_mm = 0.01  # mm per pixel
        self.image_center_x = self.image.shape[1] // 2
        self.image_center_y = self.image.shape[0] // 2
        
    def pixel_to_workspace(self, x: int, y: int) -> Tuple[float, float]:
        """Convert pixel coordinates to workspace coordinates (mm)."""
        # Simple linear transformation - adjust based on actual camera calibration
        workspace_x = (x - self.image_center_x) * self.pixel_to_mm
        workspace_y = (self.image_center_y - y) * self.pixel_to_mm  # Invert Y axis
        return workspace_x, workspace_y
    
    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse events for point selection."""
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.current_point_idx < 15:
                # Convert to workspace coordinates
                wx, wy = self.pixel_to_workspace(x, y)
                
                # For reserved points (ABW9-14), default to (0, 0)
                if self.current_point_idx >= 9:
                    wx, wy = 0.0, 0.0
                    print(f"{self.ABW_NAMES[self.current_point_idx]} - Using default (0, 0)")
                else:
                    print(f"{self.ABW_NAMES[self.current_point_idx]} - Pixel: ({x}, {y}), Workspace: ({wx:.2f}, {wy:.2f})")
                
                self.abw_points.append((wx, wy))
                self.draw_points()
                self.current_point_idx += 1
                
                if self.current_point_idx < 15:
                    print(f"Click to place {self.ABW_NAMES[self.current_point_idx]}")
                else:
                    print("All points annotated! Press 's' to save, 'r' to restart, 'q' to quit")
        
        elif event == cv2.EVENT_RBUTTONDOWN:
            # Undo last point
            if self.abw_points and self.current_point_idx > 0:
                self.abw_points.pop()
                self.current_point_idx -= 1
                self.draw_points()
                print(f"Removed last point. Click to place {self.ABW_NAMES[self.current_point_idx]}")
    
    def draw_points(self):
        """Draw all annotated points on the image."""
        self.display_image = cv2.cvtColor(self.image, cv2.COLOR_GRAY2BGR)
        
        for i, (wx, wy) in enumerate(self.abw_points):
            # Convert back to pixel coordinates for display
            px = int(wx / self.pixel_to_mm + self.image_center_x)
            py = int(self.image_center_y - wy / self.pixel_to_mm)
            
            # Different colors for different point groups
            if i < 3:  # Left side points
                color = (255, 0, 0)  # Blue
            elif i < 6:  # Bottom points
                color = (0, 255, 0)  # Green
            elif i < 9:  # Right side points
                color = (0, 0, 255)  # Red
            else:  # Reserved points
                color = (128, 128, 128)  # Gray
            
            cv2.circle(self.display_image, (px, py), 5, color, -1)
            cv2.putText(self.display_image, f"ABW{i}", (px + 10, py - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        # Draw lines connecting the points to show the joint profile
        if len(self.abw_points) >= 2:
            for i in range(len(self.abw_points) - 1):
                if i < 8:  # Only connect the main profile points
                    wx1, wy1 = self.abw_points[i]
                    wx2, wy2 = self.abw_points[i + 1]
                    px1 = int(wx1 / self.pixel_to_mm + self.image_center_x)
                    py1 = int(self.image_center_y - wy1 / self.pixel_to_mm)
                    px2 = int(wx2 / self.pixel_to_mm + self.image_center_x)
                    py2 = int(self.image_center_y - wy2 / self.pixel_to_mm)
                    cv2.line(self.display_image, (px1, py1), (px2, py2), (255, 255, 0), 1)
        
        cv2.imshow(self.window_name, self.display_image)
    
    def save_to_yaml(self, output_path: str):
        """Save annotated points to YAML format."""
        data = {
            'name': os.path.basename(self.image_path).replace('.tiff', ''),
            'image': os.path.basename(self.image_path),
            'description': 'Manually annotated test case',
            'joint_properties': {
                'upper_joint_width': 30.0,
                'left_max_surface_angle': 10.0,
                'right_max_surface_angle': 10.0,
                'left_joint_angle': 45.0,
                'right_joint_angle': 45.0,
                'groove_depth': 15.0,
                'upper_joint_width_tolerance': 7.0,
                'surface_angle_tolerance': 10.0,
                'groove_angle_tolerance': 9.0,
                'offset_distance': 3.0
            },
            'scanner_config': {
                'gray_minimum_wall': 48,
                'gray_minimum_groove': 16,
                'gray_minimum_surface': 48
            },
            'expected_result': {
                'success': True,
                'tolerance': 0.5,
                'abw_points': {}
            }
        }
        
        # Add ABW points
        for i, (x, y) in enumerate(self.abw_points):
            data['expected_result']['abw_points'][f'ABW{i}'] = {'x': round(x, 2), 'y': round(y, 2)}
        
        with open(output_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)
        
        print(f"Saved annotation to {output_path}")
    
    def run(self):
        """Run the interactive annotation tool."""
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        
        print("=== ABW Point Annotator ===")
        print("Instructions:")
        print("- Left click to place points in order (ABW0 to ABW8)")
        print("- Right click to undo last point")
        print("- Press 's' to save annotation")
        print("- Press 'r' to restart annotation")
        print("- Press 'q' to quit")
        print("- Press '+'/'-' to zoom in/out")
        print()
        print(f"Click to place {self.ABW_NAMES[0]}")
        
        self.draw_points()
        
        while True:
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord('s'):
                if len(self.abw_points) == 15:
                    output_path = self.image_path.replace('.tiff', '_annotation.yaml')
                    self.save_to_yaml(output_path)
                else:
                    print(f"Please annotate all 15 points first (current: {len(self.abw_points)})")
            elif key == ord('r'):
                self.abw_points = []
                self.current_point_idx = 0
                self.draw_points()
                print(f"Restarted. Click to place {self.ABW_NAMES[0]}")
            elif key == ord('+'):
                self.scale *= 1.2
                self.resize_display()
            elif key == ord('-'):
                self.scale /= 1.2
                self.resize_display()
        
        cv2.destroyAllWindows()
    
    def resize_display(self):
        """Resize the display image based on current scale."""
        height, width = self.image.shape
        new_width = int(width * self.scale)
        new_height = int(height * self.scale)
        resized = cv2.resize(self.display_image, (new_width, new_height))
        cv2.imshow(self.window_name, resized)


def main():
    parser = argparse.ArgumentParser(description='Annotate ABW points on test images')
    parser.add_argument('image', help='Path to the test image (TIFF format)')
    parser.add_argument('--pixel-to-mm', type=float, default=0.01,
                       help='Conversion factor from pixels to millimeters (default: 0.01)')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.image):
        print(f"Error: Image file not found: {args.image}")
        return
    
    annotator = ABWAnnotator(args.image)
    annotator.pixel_to_mm = args.pixel_to_mm
    annotator.run()


if __name__ == '__main__':
    main()