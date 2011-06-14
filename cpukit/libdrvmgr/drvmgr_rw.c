#include <drvmgr/drvmgr.h>

#ifdef DRV_MGR_BUS_RW

int drvmgr_read_io8(
	struct drvmgr_dev *dev,
	uint8_t *srcadr,
	uint8_t *result
	)
{
	struct drvmgr_bus_ops *ops;

	if ( dev->rw_dev )
		dev = dev->rw_dev;

	ops = dev->parent->ops;

	if ( !ops->read_io8 )
		return -1;
	return ops->read_io8(dev, srcadr, result);
}

int drvmgr_read_io16(
	struct drvmgr_dev *dev,
	uint16_t *srcadr,
	uint16_t *result
	)
{
	struct drvmgr_bus_ops *ops;

	if ( dev->rw_dev )
		dev = dev->rw_dev;

	ops = dev->parent->ops;

	if ( !ops->read_io16 )
		return -1;
	return ops->read_io16(dev, srcadr, result);
}

int drvmgr_read_io32(
	struct drvmgr_dev *dev,
	uint32_t *srcadr,
	uint32_t *result
	)
{
	struct drvmgr_bus_ops *ops;

	if ( dev->rw_dev )
		dev = dev->rw_dev;

	ops = dev->parent->ops;

	if ( !ops->read_io32 )
		return -1;
	return ops->read_io32(dev, srcadr, result);
}

int drvmgr_read_io64(
	struct drvmgr_dev *dev,
	uint64_t *srcadr,
	uint64_t *result
	)
{
	struct drvmgr_bus_ops *ops;

	if ( dev->rw_dev )
		dev = dev->rw_dev;

	ops = dev->parent->ops;

	if ( !ops->read_io64 )
		return -1;
	return ops->read_io64(dev, srcadr, result);
}

int drvmgr_write_io8(
	struct drvmgr_dev *dev,
	uint8_t *dstadr,
	uint8_t data
	)
{
	struct drvmgr_bus_ops *ops;

	if ( dev->rw_dev )
		dev = dev->rw_dev;

	ops = dev->parent->ops;

	if ( !ops->write_io8 )
		return -1;
	return ops->write_io8(dev, dstadr, data);
}

int drvmgr_write_io16(
	struct drvmgr_dev *dev,
	uint16_t *dstadr,
	uint16_t data
	)
{
	struct drvmgr_bus_ops *ops;

	if ( dev->rw_dev )
		dev = dev->rw_dev;

	ops = dev->parent->ops;

	if ( !ops->write_io16 )
		return -1;
	return ops->write_io16(dev, dstadr, data);
}

int drvmgr_write_io32(
	struct drvmgr_dev *dev,
	uint32_t *dstadr,
	uint32_t data
	)
{
	struct drvmgr_bus_ops *ops;

	if ( dev->rw_dev )
		dev = dev->rw_dev;

	ops = dev->parent->ops;

	if ( !ops->write_io32 )
		return -1;
	return ops->write_io32(dev, dstadr, data);
}

int drvmgr_write_io64(
	struct drvmgr_dev *dev,
	uint64_t *dstadr,
	uint64_t data
	)
{
	struct drvmgr_bus_ops *ops;

	if ( dev->rw_dev )
		dev = dev->rw_dev;

	ops = dev->parent->ops;

	if ( !ops->write_io64 )
		return -1;
	return ops->write_io64(dev, dstadr, data);
}

int drvmgr_read_mem(
	struct drvmgr_dev *dev,
	void *dest,
	const void *src,
	int n
	)
{
	struct drvmgr_bus_ops *ops;

	if ( dev->rw_dev )
		dev = dev->rw_dev;

	ops = dev->parent->ops;

	if ( !ops->read_mem )
		return -1;
	return ops->read_mem(dev, dest, src, n);
}

int drvmgr_write_mem(
	struct drvmgr_dev *dev,
	void *dest,
	const void *src,
	int n
	)
{
	struct drvmgr_bus_ops *ops;

	if ( dev->rw_dev )
		dev = dev->rw_dev;

	ops = dev->parent->ops;

	if ( !ops->write_mem )
		return -1;
	return ops->write_mem(dev, dest, src, n);
}

/* Clear a range of memory in 128 byte chunks
 * This call will take 128 bytes for buffer on stack 
 */
int drvmgr_write_memset(struct drvmgr_dev *dev, void *dstadr, int c, size_t n)
{
	unsigned long long buf[16+1]; /* Extra bytes after data are reserved for optimizations by write_mem */
	int txlen, status;
	char *adr;
	int (*write_mem)(struct drvmgr_dev *dev, void *dest, const void *src, int n);
	struct drvmgr_bus_ops *ops;

	/* Similar to drvmgr_write_mem() */
	if ( dev->rw_dev )
		dev = dev->rw_dev;
	ops = dev->parent->ops;
	if ( !ops->write_mem )
		return -1;
	write_mem = ops->write_mem;

	if ( n <= 0 )
		return 0;

	if ( n > 16*8 )
		txlen = 16*8;
	else
		txlen = n;

	memset(buf, 0, txlen);

	adr = dstadr;
	while ( n > 0 ) {

		status = write_mem(dev, adr, &buf[0], txlen);
		if ( status != 0 ) {
			return status;
		}
		adr += txlen;
		n -= txlen;

		/* next length to transmitt */
		if ( n > 16*8 )
			txlen = 16*8;
		else
			txlen = n;
	}

	return 0;
}

#endif

