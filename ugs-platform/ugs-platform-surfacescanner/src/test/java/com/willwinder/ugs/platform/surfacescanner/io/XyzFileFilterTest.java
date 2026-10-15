package com.willwinder.ugs.platform.surfacescanner.io;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.File;
import java.io.IOException;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class XyzFileFilterTest {
    @Rule
    public TemporaryFolder temporaryFolder = new TemporaryFolder();

    private final XyzFileFilter filter = new XyzFileFilter();

    @Test
    public void acceptShouldAllowXyzFilesInAnyCase() throws IOException {
        assertTrue(filter.accept(temporaryFolder.newFile("scan.xyz")));
        assertTrue(filter.accept(temporaryFolder.newFile("scan.XYZ")));
        assertTrue(filter.accept(temporaryFolder.newFile("scan.Xyz")));
    }

    @Test
    public void acceptShouldNotAllowOtherFiles() throws IOException {
        assertFalse(filter.accept(temporaryFolder.newFile("scan.txt")));
        assertFalse(filter.accept(temporaryFolder.newFile("scan.xyz.txt")));
        assertFalse(filter.accept(temporaryFolder.newFile("scanxyz")));
    }

    @Test
    public void acceptShouldAllowDirectoriesSoTheyCanBeNavigated() throws IOException {
        assertTrue(filter.accept(temporaryFolder.newFolder("scans")));
    }

    @Test
    public void withXyzExtensionShouldAddTheExtensionIfMissing() {
        File directory = new File("/home/user/scans");
        assertEquals(new File(directory, "scan.xyz"), XyzFileFilter.withXyzExtension(new File(directory, "scan")));
        assertEquals(new File(directory, "scan.txt.xyz"), XyzFileFilter.withXyzExtension(new File(directory, "scan.txt")));
        assertEquals(new File(directory, "Moms_white x-mas_cutouts.xyz"),
                XyzFileFilter.withXyzExtension(new File(directory, "Moms_white x-mas_cutouts")));
    }

    @Test
    public void withXyzExtensionShouldUseTheExtensionInLowerCase() {
        File directory = new File("/home/user/scans");
        assertEquals(new File(directory, "scan.xyz"), XyzFileFilter.withXyzExtension(new File(directory, "scan.xyz")));
        assertEquals(new File(directory, "scan.xyz"), XyzFileFilter.withXyzExtension(new File(directory, "scan.XYZ")));
        assertEquals(new File(directory, "scan.xyz"), XyzFileFilter.withXyzExtension(new File(directory, "scan.Xyz")));
    }

    @Test
    public void withXyzExtensionShouldKeepAnExistingFileSoThatItIsReplaced() throws IOException {
        File existingFile = temporaryFolder.newFile("scan.XYZ");
        assertEquals(existingFile, XyzFileFilter.withXyzExtension(existingFile));

        // A new file still gets the extension in lower case
        assertEquals(new File(temporaryFolder.getRoot(), "other.xyz"),
                XyzFileFilter.withXyzExtension(new File(temporaryFolder.getRoot(), "other.XYZ")));
    }
}
