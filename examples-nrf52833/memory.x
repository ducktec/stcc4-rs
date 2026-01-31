/* Linker script for the nRF52 - WITHOUT SOFT DEVICE */
MEMORY
{
  /* NOTE K = KiBi = 1024 bytes */
  FLASH : ORIGIN = 0x00000000, LENGTH = 512K
  RAM : ORIGIN = 0x20000000, LENGTH = 128K
}

/* defmt sections - required for defmt logging */
SECTIONS
{
  .defmt (INFO) :
  {
    /* defmt string table */
    *(.defmt.*)
  }
}
