import open3d as o3d

# === Load original PCD ===
pcd = o3d.io.read_point_cloud("uneven_world.pcd")

# === Create visualizer to save screenshot of original ===
vis_original = o3d.visualization.Visualizer()
vis_original.create_window(visible=False)
vis_original.add_geometry(pcd)
vis_original.poll_events()
vis_original.update_renderer()
vis_original.capture_screen_image("original_pcd.png")
vis_original.destroy_window()
print("Saved: original_pcd.png")

# === Apply denoising ===
# You can change nb_neighbors or std_ratio for different effects
denoised_pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

# === Save screenshot of denoised point cloud ===
vis_denoised = o3d.visualization.Visualizer()
vis_denoised.create_window(visible=False)
vis_denoised.add_geometry(denoised_pcd)
vis_denoised.poll_events()
vis_denoised.update_renderer()
vis_denoised.capture_screen_image("denoised_pcd.png")
vis_denoised.destroy_window()
print("Saved: denoised_pcd.png")

