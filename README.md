open the link !!!
https://srivatsa-dotcom.github.io/world-editor/

## Library, saved worlds, walls and merging (web editor)

- **Library** (left panel → Library): every folder, model and mesh you add is stored in your browser (IndexedDB) with a rendered picture. It is still there next time you open the page. Click a card to add it to the open world, or drag it into the view.
- **Saved worlds** (left panel → Worlds): every world you open, create or merge is kept with a picture and its latest edits. Open, download, delete, or merge one into the world that is open.
- **Draw walls**: Scene → Walls (or press `L`). Click corners on the floor, Shift for 15° steps, click the first corner to close a room, Enter to finish. Thickness, height and colour are adjustable. Walls are written as one static model with a box per wall.
- **Merge worlds**: Worlds → Merge… copies another world's models, includes and lights into the open world, with an optional move (X/Y/Z), turn and name prefix. Useful for stacking floors (e.g. Z = 3 m).
- **Broken poses**: models placed absurdly far away (for example `z = -7.5e8` in the AWS hospital world) are left out of *Fit all* and can be moved back with one click.

Storage is per browser and per device (nothing is uploaded to a server). Clearing site data in the browser removes the library.
