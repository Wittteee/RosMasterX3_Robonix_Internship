# Keep the Astra workspace visible after yahboomcar_ws replaces the environment.
if [ -f "$HOME/software/library_ws/devel/setup.sh" ]; then
  . "$HOME/software/library_ws/devel/setup.sh" --extend
fi
