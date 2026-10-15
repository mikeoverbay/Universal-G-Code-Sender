package com.willwinder.ugs.nbm.visualizer.shared;

import com.jogamp.opengl.GLAutoDrawable;
import com.willwinder.universalgcodesender.model.Position;
import com.willwinder.universalgcodesender.model.UnitUtils.Units;
import org.junit.Test;

import java.util.List;
import java.util.Optional;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;

public class RenderableBoundsTest {
    private static final RenderableBounds AREA = bounds(0, 0, 0, 10, 10, 1);
    private static final RenderableBounds OTHER_AREA = bounds(-5, 2, -1, 8, 20, 0.5);

    @Test
    public void unionShouldCoverBothAreas() {
        assertEquals(bounds(-5, 0, -1, 10, 20, 1), AREA.union(OTHER_AREA));
        assertEquals(bounds(-5, 0, -1, 10, 20, 1), OTHER_AREA.union(AREA));
    }

    @Test
    public void unionWithoutAnotherAreaShouldBeTheSameArea() {
        assertSame(AREA, AREA.union(null));
    }

    @Test
    public void areasWithTheSameCornersShouldBeEqual() {
        // The view is only fitted again when the area changes
        assertEquals(bounds(0, 0, 0, 10, 10, 1), AREA);
    }

    @Test
    public void ofShouldCombineTheEnabledRenderablesThatHaveAnArea() {
        List<Renderable> renderables = List.of(
                renderable(AREA, true),
                renderable(null, true),
                renderable(bounds(-1000, -1000, -1000, 1000, 1000, 1000), false),
                renderable(OTHER_AREA, true));

        assertEquals(bounds(-5, 0, -1, 10, 20, 1), RenderableBounds.of(renderables));
    }

    @Test
    public void ofShouldBeNullWithoutEnabledRenderablesThatHaveAnArea() {
        List<Renderable> renderables = List.of(
                renderable(null, true),
                renderable(AREA, false));

        assertNull(RenderableBounds.of(renderables));
        assertNull(RenderableBounds.of(List.of()));
    }

    private static RenderableBounds bounds(double minX, double minY, double minZ, double maxX, double maxY, double maxZ) {
        return new RenderableBounds(new Position(minX, minY, minZ, Units.MM), new Position(maxX, maxY, maxZ, Units.MM));
    }

    private static Renderable renderable(RenderableBounds bounds, boolean enabled) {
        Renderable renderable = new Renderable(1, "test", "test") {
            @Override
            public boolean rotate() {
                return false;
            }

            @Override
            public boolean center() {
                return false;
            }

            @Override
            public void init(GLAutoDrawable drawable) {
            }

            @Override
            public void draw(GLAutoDrawable drawable, boolean idle, Position machineCoord, Position workCoord, Position objectMin, Position objectMax, double scaleFactor, Position mouseWorldCoordinates, Position rotation) {
            }

            @Override
            public Optional<RenderableBounds> getBounds() {
                return Optional.ofNullable(bounds);
            }
        };
        renderable.isEnabled = enabled;
        return renderable;
    }
}
