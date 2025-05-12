import cv2
import os
from pathlib import Path
import argparse
from tqdm import tqdm

class VideoFrameConverter:
    def __init__(self, target_fps=2):
        """
        Initialize video frame rate converter
        
        Args:
            target_fps: Target frames per second (default: 2)
        """
        self.target_fps = target_fps

    def convert_video(self, input_path, output_path=None, show_progress=True):
        """
        Convert video to target frame rate
        
        Args:
            input_path: Path to input video
            output_path: Path to save converted video (default: auto-generate)
            show_progress: Whether to show progress bar
        """
        # Handle output path
        if output_path is None:
            input_path = Path(input_path)
            output_path = str(input_path.parent / f"{input_path.stem}_2fps{input_path.suffix}")

        # Open input video
        cap = cv2.VideoCapture(input_path)
        if not cap.isOpened():
            raise ValueError(f"Cannot open video file: {input_path}")

        # Get video properties
        original_fps = int(cap.get(cv2.CAP_PROP_FPS))
        frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        print(f"Original video properties:")
        print(f"- FPS: {original_fps}")
        print(f"- Total frames: {frame_count}")
        print(f"- Resolution: {width}x{height}")

        # Calculate frame interval
        frame_interval = original_fps / self.target_fps
        
        # Create video writer
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(output_path, fourcc, self.target_fps, (width, height))

        frame_idx = 0
        saved_count = 0

        # Setup progress bar
        if show_progress:
            pbar = tqdm(total=frame_count, desc="Converting video")

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break

                # Save frame if it matches the target interval
                if frame_idx % frame_interval < 1:
                    out.write(frame)
                    saved_count += 1

                frame_idx += 1
                if show_progress:
                    pbar.update(1)

        finally:
            if show_progress:
                pbar.close()
            cap.release()
            out.release()

        # Calculate actual achieved FPS
        actual_fps = saved_count / (frame_count / original_fps)
        
        print(f"\nConversion completed:")
        print(f"- Output path: {output_path}")
        print(f"- Frames saved: {saved_count}")
        print(f"- Target FPS: {self.target_fps}")
        print(f"- Actual FPS: {actual_fps:.2f}")

    def process_directory(self, input_dir, output_dir=None, extensions=('.mp4', '.avi', '.mov')):
        """
        Convert all videos in a directory
        
        Args:
            input_dir: Input directory containing videos
            output_dir: Output directory for converted videos
            extensions: Tuple of valid video extensions
        """
        input_dir = Path(input_dir)
        if output_dir is None:
            output_dir = input_dir / "2fps_converted"
        else:
            output_dir = Path(output_dir)
        
        output_dir.mkdir(exist_ok=True)
        
        # Find all video files
        video_files = []
        for ext in extensions:
            video_files.extend(input_dir.glob(f"*{ext}"))
        
        if not video_files:
            print(f"No video files found in {input_dir}")
            return
        
        print(f"Found {len(video_files)} video files")
        
        # Process each video
        for video_file in video_files:
            output_path = output_dir / f"{video_file.stem}_2fps{video_file.suffix}"
            print(f"\nProcessing: {video_file.name}")
            try:
                self.convert_video(str(video_file), str(output_path))
            except Exception as e:
                print(f"Error processing {video_file.name}: {e}")

def main():
    parser = argparse.ArgumentParser(description='Video Frame Rate Converter')
    parser.add_argument('--input', type=str,default=r'D:\XJTLU\MEC202\videos\8.mp4',
                        help='Input video file or directory')
    parser.add_argument('--output', type=str, default=r'D:\XJTLU\MEC202\videos\8_.mp4',
                        help='Output path (optional)')
    parser.add_argument('--fps', type=float, default=2,
                        help='Target FPS (default: 2)')
    parser.add_argument('--batch', action='store_true',
                        help='Process entire directory')
    
    args = parser.parse_args()
    
    converter = VideoFrameConverter(target_fps=args.fps)
    
    try:
        if args.batch:
            converter.process_directory(args.input, args.output)
        else:
            converter.convert_video(args.input, args.output)
            
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()