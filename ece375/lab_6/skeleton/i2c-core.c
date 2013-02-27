/* i2c-core.c - a device driver for the iic-bus interface		     */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 1995-99 Simon G. Vogl

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.		     */
/* ------------------------------------------------------------------------- */

/* With some changes from Kyösti Mälkki <kmalkki@cc.hut.fi>.
   All SMBus-related things are written by Frodo Looijaard <frodol@dds.nl>
   SMBus 2.0 support by Mark Studebaker <mdsxyz123@yahoo.com> and
   Jean Delvare <khali@linux-fr.org> */

/* $Id$ */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/init.h>
#include "i2c.h"
#include <asm/uaccess.h>

/* ----- global defines ---------------------------------------------------- */

/* exclusive access to the bus */
#define I2C_LOCK(adap) down(&adap->lock)
#define I2C_UNLOCK(adap) up(&adap->lock) 

#define DEB(x) if (i2c_debug>=1) x;
#define DEB2(x) if (i2c_debug>=2) x;

/* ----- global variables -------------------------------------------------- */

DECLARE_MUTEX(core_lists);
static struct i2c_adapter *adapters[I2C_ADAP_MAX];
static struct i2c_driver *drivers[I2C_DRIVER_MAX];

/**** debug level */
static int i2c_debug;

/* ---------------------------------------------------
 * /proc entry declarations
 *----------------------------------------------------
 */

#ifdef CONFIG_PROC_FS
static ssize_t i2cproc_bus_read(struct file * file, char * buf,size_t count, 
                                loff_t *ppos);
static int read_bus_i2c(char *buf, char **start, off_t offset, int len,
                           int *eof , void *private);

/* To implement the dynamic /proc/bus/i2c-? files, we need our own 
   implementation of the read hook */
static struct file_operations i2cproc_operations = {
	.read		= i2cproc_bus_read,
};

static int i2cproc_register(struct i2c_adapter *adap, int bus);
static void i2cproc_remove(int bus);

#endif /* CONFIG_PROC_FS */


/* ---------------------------------------------------
 * registering functions 
 * --------------------------------------------------- 
 */

/* -----
 * i2c_add_adapter is called from within the algorithm layer,
 * when a new hw adapter registers. A new device is register to be
 * available for clients.
 */
int i2c_add_adapter(struct i2c_adapter *adap)
{
	int i,j,res = 0;

	down(&core_lists);
	for (i = 0; i < I2C_ADAP_MAX; i++)
		if (NULL == adapters[i])
			break;
	if (I2C_ADAP_MAX == i) {
		printk(KERN_WARNING 
		       " i2c-core.o: register_adapter(%s) - enlarge I2C_ADAP_MAX.\n",
			adap->name);
		res = -ENOMEM;
		goto ERROR0;
	}
	
#ifdef CONFIG_PROC_FS
	res = i2cproc_register(adap, i);
	if (res<0)
	    goto ERROR0;
#endif /* def CONFIG_PROC_FS */

	adapters[i] = adap;
	
	/* init data types */
	init_MUTEX(&adap->lock);

	/* inform drivers of new adapters */
	for (j=0;j<I2C_DRIVER_MAX;j++)
		if (drivers[j]!=NULL && 
		    (drivers[j]->flags&(I2C_DF_NOTIFY|I2C_DF_DUMMY)))
			/* We ignore the return code; if it fails, too bad */
			drivers[j]->attach_adapter(adap);
	
	DEB(printk(KERN_DEBUG "i2c-core.o: adapter %s registered as adapter %d.\n",
	           adap->name,i));
ERROR0:
	up(&core_lists);
	return res;
}


int i2c_del_adapter(struct i2c_adapter *adap)
{
	int i,j,res = 0;

	down(&core_lists);
	for (i = 0; i < I2C_ADAP_MAX; i++)
		if (adap == adapters[i])
			break;
	if (I2C_ADAP_MAX == i) {
		printk(KERN_WARNING "i2c-core.o: unregister_adapter adap [%s] not found.\n",
			adap->name);
		res = -ENODEV;
		goto ERROR0;
	}

	/* DUMMY drivers do not register their clients, so we have to
	 * use a trick here: we call driver->attach_adapter to
	 * *detach* it! Of course, each dummy driver should know about
	 * this or hell will break loose...
	 */
	for (j = 0; j < I2C_DRIVER_MAX; j++) 
		if (drivers[j] && (drivers[j]->flags & I2C_DF_DUMMY))
			if ((res = drivers[j]->attach_adapter(adap))) {
				printk(KERN_WARNING "i2c-core.o: can't detach adapter %s "
				       "while detaching driver %s: driver not "
				       "detached!\n", adap->name, drivers[j]->name);
				goto ERROR0;
			}

	/* detach any active clients. This must be done first, because
	 * it can fail; in which case we give up. */
	for (j=0;j<I2C_CLIENT_MAX;j++) {
		struct i2c_client *client = adap->clients[j];
		if (client!=NULL)
		    /* detaching devices is unconditional of the set notify
		     * flag, as _all_ clients that reside on the adapter
		     * must be deleted, as this would cause invalid states.
		     */
			if ((res=client->driver->detach_client(client))) {
				printk(KERN_ERR "i2c-core.o: adapter %s not "
					"unregistered, because client at "
					"address %02x can't be detached\n",
					adap->name, client->addr);
				goto ERROR0;
			}
	}

#ifdef CONFIG_PROC_FS
	i2cproc_remove(i);
#endif /* def CONFIG_PROC_FS */

	adapters[i] = NULL;
	DEB(printk(KERN_DEBUG "i2c-core.o: adapter unregistered: %s\n",adap->name));
ERROR0:
	up(&core_lists);
	return res;
}


/* -----
 * What follows is the "upwards" interface: commands for talking to clients,
 * which implement the functions to access the physical information of the
 * chips.
 */

int i2c_add_driver(struct i2c_driver *driver)
{
	int i;

	down(&core_lists);
	for (i = 0; i < I2C_DRIVER_MAX; i++)
		if (NULL == drivers[i])
			break;
	if (I2C_DRIVER_MAX == i) {
		printk(KERN_WARNING 
		       " i2c-core.o: register_driver(%s) "
		       "- enlarge I2C_DRIVER_MAX.\n",
			driver->name);
		up(&core_lists);
		return -ENOMEM;
	}
	drivers[i] = driver;
	DEB(printk(KERN_DEBUG "i2c-core.o: driver %s registered.\n",driver->name));
	
	/* now look for instances of driver on our adapters
	 */
	if (driver->flags& (I2C_DF_NOTIFY|I2C_DF_DUMMY)) {
		for (i=0;i<I2C_ADAP_MAX;i++)
			if (adapters[i]!=NULL)
				/* Ignore errors */
				driver->attach_adapter(adapters[i]);
	}
	up(&core_lists);
	return 0;
}

int i2c_del_driver(struct i2c_driver *driver)
{
	int i,j,k,res = 0;

	down(&core_lists);
	for (i = 0; i < I2C_DRIVER_MAX; i++)
		if (driver == drivers[i])
			break;
	if (I2C_DRIVER_MAX == i) {
		printk(KERN_WARNING " i2c-core.o: unregister_driver: "
				    "[%s] not found\n",
			driver->name);
		up(&core_lists);
		return -ENODEV;
	}
	/* Have a look at each adapter, if clients of this driver are still
	 * attached. If so, detach them to be able to kill the driver 
	 * afterwards.
	 */
	DEB2(printk(KERN_DEBUG "i2c-core.o: unregister_driver - looking for clients.\n"));
	/* removing clients does not depend on the notify flag, else 
	 * invalid operation might (will!) result, when using stale client
	 * pointers.
	 */
	for (k=0;k<I2C_ADAP_MAX;k++) {
		struct i2c_adapter *adap = adapters[k];
		if (adap == NULL) /* skip empty entries. */
			continue;
		DEB2(printk(KERN_DEBUG "i2c-core.o: examining adapter %s:\n",
			    adap->name));
		if (driver->flags & I2C_DF_DUMMY) {
		/* DUMMY drivers do not register their clients, so we have to
		 * use a trick here: we call driver->attach_adapter to
		 * *detach* it! Of course, each dummy driver should know about
		 * this or hell will break loose...  
		 */
			if ((res = driver->attach_adapter(adap))) {
				printk(KERN_WARNING "i2c-core.o: while unregistering "
				       "dummy driver %s, adapter %s could "
				       "not be detached properly; driver "
				       "not unloaded!\n", driver->name,
				       adap->name);
				goto ERROR0;
			}
		} else {
			for (j=0;j<I2C_CLIENT_MAX;j++) { 
				struct i2c_client *client = adap->clients[j];
				if (client != NULL && 
				    client->driver == driver) {
					DEB2(printk(KERN_DEBUG "i2c-core.o: "
						    "detaching client %s:\n",
					            client->name));
					if ((res = driver->
							detach_client(client)))
					{
						printk(KERN_ERR "i2c-core.o: while "
						       "unregistering driver "
						       "`%s', the client at "
						       "address %02x of "
						       "adapter `%s' could not "
						       "be detached; driver "
						       "not unloaded!\n",
						       driver->name,
						       client->addr,
						       adap->name);
						goto ERROR0;
					}
				}
			}
		}
	}
	drivers[i] = NULL;
	DEB(printk(KERN_DEBUG "i2c-core.o: driver unregistered: %s\n",driver->name));

ERROR0:
	up(&core_lists);
	return res;
}

static int __i2c_check_addr (struct i2c_adapter *adapter, int addr)
{
	int i;
	for (i = 0; i < I2C_CLIENT_MAX ; i++) 
		if (adapter->clients[i] && (adapter->clients[i]->addr == addr))
			return -EBUSY;
	return 0;
}

int i2c_check_addr (struct i2c_adapter *adapter, int addr)
{
	int rval;

	I2C_LOCK(adapter);
	rval = __i2c_check_addr(adapter, addr);
	I2C_UNLOCK(adapter);

	return rval;
}

int i2c_attach_client(struct i2c_client *client)
{
	struct i2c_adapter *adapter = client->adapter;
	int i;

	if (i2c_check_addr(client->adapter,client->addr))
		return -EBUSY;

	I2C_LOCK(adapter);
	for (i = 0; i < I2C_CLIENT_MAX; i++)
		if (NULL == adapter->clients[i])
			break;
	if (I2C_CLIENT_MAX == i) {
		printk(KERN_WARNING 
		       " i2c-core.o: attach_client(%s) - enlarge I2C_CLIENT_MAX.\n",
			client->name);
		I2C_UNLOCK(adapter);
		return -ENOMEM;
	}
	adapter->clients[i] = client;
	I2C_UNLOCK(adapter);
	
	if (adapter->client_register) 
		if (adapter->client_register(client)) 
			printk(KERN_DEBUG "i2c-core.o: warning: client_register seems "
			       "to have failed for client %02x at adapter %s\n",
			       client->addr,adapter->name);
	DEB(printk(KERN_DEBUG "i2c-core.o: client [%s] registered to adapter [%s](pos. %d).\n",
		client->name, adapter->name,i));

	if(client->flags & I2C_CLIENT_ALLOW_USE)
		client->usage_count = 0;
	
	return 0;
}


int i2c_detach_client(struct i2c_client *client)
{
	struct i2c_adapter *adapter = client->adapter;
	int i,res;

	if( (client->flags & I2C_CLIENT_ALLOW_USE) && 
	    (client->usage_count>0))
		return -EBUSY;
	
	if (adapter->client_unregister != NULL) 
		if ((res = adapter->client_unregister(client))) {
			printk(KERN_ERR "i2c-core.o: client_unregister [%s] failed, "
			       "client not detached\n", client->name);
			return res;
		}

	I2C_LOCK(adapter);
	for (i = 0; i < I2C_CLIENT_MAX; i++)
		if (client == adapter->clients[i])
			break;
	if (I2C_CLIENT_MAX == i) {
		printk(KERN_WARNING " i2c-core.o: unregister_client "
				    "[%s] not found\n",
			client->name);
		I2C_UNLOCK(adapter);
		return -ENODEV;
	}
	adapter->clients[i] = NULL;
	I2C_UNLOCK(adapter);

	DEB(printk(KERN_DEBUG "i2c-core.o: client [%s] unregistered.\n",client->name));
	return 0;
}

static void i2c_inc_use_client(struct i2c_client *client)
{
	if (client->driver->inc_use != NULL)
		client->driver->inc_use(client);
	if (client->adapter->inc_use != NULL)
		client->adapter->inc_use(client->adapter);
}

static void i2c_dec_use_client(struct i2c_client *client)
{
	if (client->driver->dec_use != NULL)
		client->driver->dec_use(client);
	if (client->adapter->dec_use != NULL)
		client->adapter->dec_use(client->adapter);
}

struct i2c_client *i2c_get_client(int driver_id, int adapter_id, 
					struct i2c_client *prev)
{
	int i,j;
	
	/* Will iterate through the list of clients in each adapter of adapters-list
	   in search for a client that matches the search criteria. driver_id or 
	   adapter_id are ignored if set to 0. If both are ignored this returns 
	   first client found. */
	
	i = j = 0;  
	
	/* set starting point */ 
	if(prev)
	{
		if(!(prev->adapter))
			return (struct i2c_client *) -EINVAL;
		
		for(j=0; j < I2C_ADAP_MAX; j++)
			if(prev->adapter == adapters[j])
				break;
		
		/* invalid starting point? */
		if (I2C_ADAP_MAX == j) {
			printk(KERN_WARNING " i2c-core.o: get_client adapter for client:[%s] not found\n",
				prev->name);
			return (struct i2c_client *) -ENODEV;
		}	
		
		for(i=0; i < I2C_CLIENT_MAX; i++)
			if(prev == adapters[j]->clients[i])
				break;
		
		/* invalid starting point? */
		if (I2C_CLIENT_MAX == i) {
			printk(KERN_WARNING " i2c-core.o: get_client client:[%s] not found\n",
				prev->name);
			return (struct i2c_client *) -ENODEV;
		}	
		
		i++; /* start from one after prev */
	}
	
	for(; j < I2C_ADAP_MAX; j++)
	{
		if(!adapters[j])
			continue;
			
		if(adapter_id && (adapters[j]->id != adapter_id))
			continue;
		
		for(; i < I2C_CLIENT_MAX; i++)
		{
			if(!adapters[j]->clients[i])
				continue;
				
			if(driver_id && (adapters[j]->clients[i]->driver->id != driver_id))
				continue;
			if(adapters[j]->clients[i]->flags & I2C_CLIENT_ALLOW_USE)	
				return adapters[j]->clients[i];
		}
		i = 0;
	}

	return 0;
}

int i2c_use_client(struct i2c_client *client)
{
	if (client->flags & I2C_CLIENT_ALLOW_USE) {
		if (client->flags & I2C_CLIENT_ALLOW_MULTIPLE_USE)
			client->usage_count++;
		else if (client->usage_count > 0)
			return -EBUSY;
		else
			client->usage_count++;
	}

	i2c_inc_use_client(client);

	return 0;
}

int i2c_release_client(struct i2c_client *client)
{
	if(client->flags & I2C_CLIENT_ALLOW_USE) {
		if(client->usage_count>0)
			client->usage_count--;
		else
		{
			printk(KERN_WARNING " i2c-core.o: dec_use_client used one too many times\n");
			return -EPERM;
		}
	}
	
	i2c_dec_use_client(client);
	
	return 0;
}

/* ----------------------------------------------------
 * The /proc functions
 * ----------------------------------------------------
 */

#ifdef CONFIG_PROC_FS

/* This function generates the output for /proc/bus/i2c */
static int read_bus_i2c(char *buf, char **start, off_t offset, int len, int *eof, 
                 void *private)
{
	int i;
	int nr = 0;
	/* Note that it is safe to write a `little' beyond len. Yes, really. */
	down(&core_lists);
	for (i = 0; (i < I2C_ADAP_MAX) && (nr < len); i++)
		if (adapters[i]) {
			nr += sprintf(buf+nr, "i2c-%d\t", i);
			if (adapters[i]->algo->smbus_xfer) {
				if (adapters[i]->algo->master_xfer)
					nr += sprintf(buf+nr,"smbus/i2c");
				else
					nr += sprintf(buf+nr,"smbus    ");
			} else if (adapters[i]->algo->master_xfer)
				nr += sprintf(buf+nr,"i2c       ");
			else
				nr += sprintf(buf+nr,"dummy     ");
			nr += sprintf(buf+nr,"\t%-32s\t%-32s\n",
			              adapters[i]->name,
			              adapters[i]->algo->name);
		}
	up(&core_lists);
	return nr;
}

/* This function generates the output for /proc/bus/i2c-? */
ssize_t i2cproc_bus_read(struct file * file, char * buf,size_t count, 
                         loff_t *ppos)
{
	struct inode * inode = file->f_dentry->d_inode;
	char *kbuf;
	struct i2c_client *client;
	struct i2c_adapter *adap;
	int i,j,k,order_nr,len=0;
	size_t len_total;
	int order[I2C_CLIENT_MAX];
#define OUTPUT_LENGTH_PER_LINE 70

	len_total = file->f_pos + count;
	if (len_total > (I2C_CLIENT_MAX * OUTPUT_LENGTH_PER_LINE) )
		/* adjust to maximum file size */
		len_total = (I2C_CLIENT_MAX * OUTPUT_LENGTH_PER_LINE);

	down(&core_lists);
	/* adap = file->private_data; ?? --km */
	for (i = 0; i < I2C_ADAP_MAX; i++) {
	    adap = adapters[i];
	    if (adap && (adap->inode == inode->i_ino))
		break;
	}
	if ( I2C_ADAP_MAX == i ) {
	    up(&core_lists);
	    return -ENOENT;
	}

	/* We need a bit of slack in the kernel buffer; this makes the
	   sprintf safe. */
	if (! (kbuf = kmalloc(len_total +
			      OUTPUT_LENGTH_PER_LINE,
			      GFP_KERNEL)))
	    return -ENOMEM;

	/* Order will hold the indexes of the clients
	   sorted by address */
	order_nr=0;
	I2C_LOCK(adap);
	for (j = 0; j < I2C_CLIENT_MAX; j++) {
	    if ((client = adap->clients[j]) && 
		(client->driver->id != I2C_DRIVERID_I2CDEV))  {
		for(k = order_nr; 
		    (k > 0) && 
			adap->clients[order[k-1]]->
			addr > client->addr; 
		    k--)
		    order[k] = order[k-1];
		order[k] = j;
		order_nr++;
	    }
	}


	for (j = 0; (j < order_nr) && (len < len_total); j++) {
	    client = adap->clients[order[j]];
	    len += sprintf(kbuf+len,"%02x\t%-32s\t%-32s\n",
			   client->addr,
			   client->name,
			   client->driver->name);
	}
	I2C_UNLOCK(adap);
	up(&core_lists);
	
	len = len - file->f_pos;
	if (len > count)
	    len = count;
	if (len < 0) 
	    len = 0;
	if (copy_to_user (buf,kbuf+file->f_pos, len)) {
	    kfree(kbuf);
	    return -EFAULT;
	}
	file->f_pos += len;
	kfree(kbuf);
	return len;
}

static int i2cproc_register(struct i2c_adapter *adap, int bus)
{
	char name[8];
	struct proc_dir_entry *proc_entry;

	sprintf(name,"i2c-%d", bus);
	proc_entry = create_proc_entry(name,0,proc_bus);
	if (! proc_entry) {
		printk(KERN_ERR "i2c-core.o: Could not create /proc/bus/%s\n",
		       name);
		return -ENOENT;
	}
	    
	proc_entry->proc_fops = &i2cproc_operations;
	proc_entry->owner = THIS_MODULE;
	adap->inode = proc_entry->low_ino;
	return 0;
}

static void i2cproc_remove(int bus)
{
	char name[8];
	sprintf(name,"i2c-%d", bus);
	remove_proc_entry(name, proc_bus);
}

static int __init i2cproc_init(void)
{
	struct proc_dir_entry *proc_bus_i2c;

	proc_bus_i2c = create_proc_entry("i2c",0,proc_bus);
	if (!proc_bus_i2c) {
		printk(KERN_ERR "i2c-core.o: Could not create /proc/bus/i2c");
		return -ENOENT;
 	}

	proc_bus_i2c->read_proc = &read_bus_i2c;
	proc_bus_i2c->owner = THIS_MODULE;
	return 0;
}

static void __exit i2cproc_cleanup(void)
{
	remove_proc_entry("i2c",proc_bus);
}

#endif /* def CONFIG_PROC_FS */

/* ----------------------------------------------------
 * the functional interface to the i2c busses.
 * ----------------------------------------------------
 */

int i2c_transfer(struct i2c_adapter * adap, struct i2c_msg *msgs,int num)
{
	int ret;

	if (adap->algo->master_xfer) {
		if (i2c_debug >= 2) {
 	 		printk(KERN_DEBUG "i2c-core.o: master_xfer: %s with "
			       "%d msgs\n", adap->name, num);
			for (ret = 0; ret < num; ret++) {
				printk(KERN_DEBUG "i2c-core.o: "
				       "master_xfer[%d]: %c, "
				       "addr=0x%02x, len=%d\n", ret,
				       msgs[ret].flags & I2C_M_RD ? 'R' : 'W',
				       msgs[ret].addr, msgs[ret].len);
			}
		}

		I2C_LOCK(adap);
		ret = adap->algo->master_xfer(adap,msgs,num);
		I2C_UNLOCK(adap);

		return ret;
	} else {
		printk(KERN_ERR "i2c-core.o: I2C adapter %04x: I2C level transfers not supported\n",
		       adap->id);
		return -ENOSYS;
	}
}

int i2c_master_send(struct i2c_client *client,const char *buf ,int count)
{
	int ret;
	struct i2c_adapter *adap=client->adapter;
	struct i2c_msg msg;

	if (client->adapter->algo->master_xfer) {
		msg.addr   = client->addr;
		msg.flags = client->flags & I2C_M_TEN;
		msg.len = count;
		msg.buf = (char *)buf;
	
		DEB2(printk(KERN_DEBUG "i2c-core.o: master_send: writing %d bytes on %s.\n",
			count,client->adapter->name));
	
		I2C_LOCK(adap);
		ret = adap->algo->master_xfer(adap,&msg,1);
		I2C_UNLOCK(adap);

		/* if everything went ok (i.e. 1 msg transmitted), return #bytes
		 * transmitted, else error code.
		 */
		return (ret == 1 )? count : ret;
	} else {
		printk(KERN_ERR "i2c-core.o: I2C adapter %04x: I2C level transfers not supported\n",
		       client->adapter->id);
		return -ENOSYS;
	}
}

int i2c_master_recv(struct i2c_client *client, char *buf ,int count)
{
	struct i2c_adapter *adap=client->adapter;
	struct i2c_msg msg;
	int ret;
	if (client->adapter->algo->master_xfer) {
		msg.addr   = client->addr;
		msg.flags = client->flags & I2C_M_TEN;
		msg.flags |= I2C_M_RD;
		msg.len = count;
		msg.buf = buf;

		DEB2(printk(KERN_DEBUG "i2c-core.o: master_recv: reading %d bytes on %s.\n",
			count,client->adapter->name));
	
		I2C_LOCK(adap);
		ret = adap->algo->master_xfer(adap,&msg,1);
		I2C_UNLOCK(adap);
	
		DEB2(printk(KERN_DEBUG "i2c-core.o: master_recv: return:%d (count:%d, addr:0x%02x)\n",
			ret, count, client->addr));
	
		/* if everything went ok (i.e. 1 msg transmitted), return #bytes
	 	* transmitted, else error code.
	 	*/
		return (ret == 1 )? count : ret;
	} else {
		printk(KERN_ERR "i2c-core.o: I2C adapter %04x: I2C level transfers not supported\n",
		       client->adapter->id);
		return -ENOSYS;
	}
}


int i2c_control(struct i2c_client *client,
	unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct i2c_adapter *adap = client->adapter;

	DEB2(printk(KERN_DEBUG "i2c-core.o: i2c ioctl, cmd: 0x%x, arg: %#lx\n", cmd, arg));
	switch ( cmd ) {
		case I2C_RETRIES:
			adap->retries = arg;
			break;
		case I2C_TIMEOUT:
			adap->timeout = arg;
			break;
		default:
			if (adap->algo->algo_control!=NULL)
				ret = adap->algo->algo_control(adap,cmd,arg);
	}
	return ret;
}

/* ----------------------------------------------------
 * the i2c address scanning function
 * Will not work for 10-bit addresses!
 * ----------------------------------------------------
 */
int i2c_probe(struct i2c_adapter *adapter,
                   struct i2c_client_address_data *address_data,
                   i2c_client_found_addr_proc *found_proc)
{
	int addr,i,found,err;
	int adap_id = i2c_adapter_id(adapter);

	/* Forget it if we can't probe using SMBUS_QUICK */
	if (! i2c_check_functionality(adapter,I2C_FUNC_SMBUS_QUICK))
		return -1;

	for (addr = 0x00; addr <= 0x7f; addr++) {

		/* Skip if already in use */
		if (i2c_check_addr(adapter,addr))
			continue;

		/* If it is in one of the force entries, we don't do any detection
		   at all */
		found = 0;

		for (i = 0; !found && (address_data->force[i] != I2C_CLIENT_END); i += 2) {
			if (((adap_id == address_data->force[i]) || 
			     (address_data->force[i] == ANY_I2C_BUS)) &&
			     (addr == address_data->force[i+1])) {
				DEB2(printk(KERN_DEBUG "i2c-core.o: found force parameter for adapter %d, addr %04x\n",
				            adap_id,addr));
				if ((err = found_proc(adapter,addr,0,0)))
					return err;
				found = 1;
			}
		}
		if (found) 
			continue;

		/* If this address is in one of the ignores, we can forget about
		   it right now */
		for (i = 0;
		     !found && (address_data->ignore[i] != I2C_CLIENT_END);
		     i += 2) {
			if (((adap_id == address_data->ignore[i]) || 
			    ((address_data->ignore[i] == ANY_I2C_BUS))) &&
			    (addr == address_data->ignore[i+1])) {
				DEB2(printk(KERN_DEBUG "i2c-core.o: found ignore parameter for adapter %d, "
				     "addr %04x\n", adap_id ,addr));
				found = 1;
			}
		}
		for (i = 0;
		     !found && (address_data->ignore_range[i] != I2C_CLIENT_END);
		     i += 3) {
			if (((adap_id == address_data->ignore_range[i]) ||
			    ((address_data->ignore_range[i]==ANY_I2C_BUS))) &&
			    (addr >= address_data->ignore_range[i+1]) &&
			    (addr <= address_data->ignore_range[i+2])) {
				DEB2(printk(KERN_DEBUG "i2c-core.o: found ignore_range parameter for adapter %d, "
				            "addr %04x\n", adap_id,addr));
				found = 1;
			}
		}
		if (found) 
			continue;

		/* Now, we will do a detection, but only if it is in the normal or 
		   probe entries */  
		for (i = 0;
		     !found && (address_data->normal_i2c[i] != I2C_CLIENT_END);
		     i += 1) {
			if (addr == address_data->normal_i2c[i]) {
				found = 1;
				DEB2(printk(KERN_DEBUG "i2c-core.o: found normal i2c entry for adapter %d, "
				            "addr %02x\n", adap_id, addr));
			}
		}

		for (i = 0;
		     !found && (address_data->normal_i2c_range[i] != I2C_CLIENT_END);
		     i += 2) {
			if ((addr >= address_data->normal_i2c_range[i]) &&
			    (addr <= address_data->normal_i2c_range[i+1])) {
				found = 1;
				DEB2(printk(KERN_DEBUG "i2c-core.o: found normal i2c_range entry for adapter %d, "
				            "addr %04x\n", adap_id,addr));
			}
		}

		for (i = 0;
		     !found && (address_data->probe[i] != I2C_CLIENT_END);
		     i += 2) {
			if (((adap_id == address_data->probe[i]) ||
			    ((address_data->probe[i] == ANY_I2C_BUS))) &&
			    (addr == address_data->probe[i+1])) {
				found = 1;
				DEB2(printk(KERN_DEBUG "i2c-core.o: found probe parameter for adapter %d, "
				            "addr %04x\n", adap_id,addr));
			}
		}
		for (i = 0;
		     !found && (address_data->probe_range[i] != I2C_CLIENT_END);
		     i += 3) {
			if (((adap_id == address_data->probe_range[i]) ||
			   (address_data->probe_range[i] == ANY_I2C_BUS)) &&
			   (addr >= address_data->probe_range[i+1]) &&
			   (addr <= address_data->probe_range[i+2])) {
				found = 1;
				DEB2(printk(KERN_DEBUG "i2c-core.o: found probe_range parameter for adapter %d, "
				            "addr %04x\n", adap_id,addr));
			}
		}
		if (!found) 
			continue;

		/* OK, so we really should examine this address. First check
		   whether there is some client here at all! */
		if (i2c_smbus_xfer(adapter,addr,0,0,0,I2C_SMBUS_QUICK,NULL) >= 0)
			if ((err = found_proc(adapter,addr,0,-1)))
				return err;
	}
	return 0;
}

/*
 * return id number for a specific adapter
 */
int i2c_adapter_id(struct i2c_adapter *adap)
{
	int i;
	for (i = 0; i < I2C_ADAP_MAX; i++)
		if (adap == adapters[i])
			return i;
	return -1;
}

/* The SMBus parts */

#define POLY    (0x1070U << 3) 
static u8
crc8(u16 data)
{
	int i;
  
	for(i = 0; i < 8; i++) {
		if (data & 0x8000) 
			data = data ^ POLY;
		data = data << 1;
	}
	return (u8)(data >> 8);
}

/* Incremental CRC8 over count bytes in the array pointed to by p */
static u8 i2c_smbus_pec(u8 crc, u8 *p, size_t count)
{
	int i;

	for(i = 0; i < count; i++)
		crc = crc8((crc ^ p[i]) << 8);
	return crc;
}

/* Assume a 7-bit address, which is reasonable for SMBus */
static u8 i2c_smbus_msg_pec(u8 pec, struct i2c_msg *msg)
{
	/* The address will be sent first */
	u8 addr = (msg->addr << 1) | !!(msg->flags & I2C_M_RD);
	pec = i2c_smbus_pec(pec, &addr, 1);

	/* The data buffer follows */
	return i2c_smbus_pec(pec, msg->buf, msg->len);
}

/* Used for write only transactions */
static inline void i2c_smbus_add_pec(struct i2c_msg *msg)
{
	msg->buf[msg->len] = i2c_smbus_msg_pec(0, msg);
	msg->len++;
}

/* Return <0 on CRC error
   If there was a write before this read (most cases) we need to take the
   partial CRC from the write part into account.
   Note that this function does modify the message (we need to decrease the
   message length to hide the CRC byte from the caller). */
int i2c_smbus_check_pec(u8 cpec, struct i2c_msg *msg)
{
	u8 rpec = msg->buf[--msg->len];
	cpec = i2c_smbus_msg_pec(cpec, msg);

	if(rpec != cpec) {
		DEB(printk(KERN_DEBUG "i2c-core.o: Bad PEC 0x%02x vs. 0x%02x\n",
		           rpec, cpec));
		return -1;
	}
	return 0;	
}

extern s32 i2c_smbus_write_quick(struct i2c_client * client, u8 value)
{
	return i2c_smbus_xfer(client->adapter,client->addr,client->flags,
 	                      value,0,I2C_SMBUS_QUICK,NULL);
}

extern s32 i2c_smbus_read_byte(struct i2c_client * client)
{
	union i2c_smbus_data data;
	if (i2c_smbus_xfer(client->adapter,client->addr,client->flags,
	                   I2C_SMBUS_READ,0,I2C_SMBUS_BYTE, &data))
		return -1;
	else
		return data.byte;
}

extern s32 i2c_smbus_write_byte(struct i2c_client * client, u8 value)
{
	return i2c_smbus_xfer(client->adapter,client->addr,client->flags,
	                      I2C_SMBUS_WRITE, value, I2C_SMBUS_BYTE, NULL);
}

extern s32 i2c_smbus_read_byte_data(struct i2c_client * client, u8 command)
{
	union i2c_smbus_data data;
	if (i2c_smbus_xfer(client->adapter,client->addr,client->flags,
	                   I2C_SMBUS_READ,command, I2C_SMBUS_BYTE_DATA,&data))
		return -1;
	else
		return data.byte;
}

extern s32 i2c_smbus_write_byte_data(struct i2c_client * client, u8 command,
                                     u8 value)
{
	union i2c_smbus_data data;
	data.byte = value;
	return i2c_smbus_xfer(client->adapter,client->addr,client->flags,
	                      I2C_SMBUS_WRITE,command,
	                      I2C_SMBUS_BYTE_DATA,&data);
}

extern s32 i2c_smbus_read_word_data(struct i2c_client * client, u8 command)
{
	union i2c_smbus_data data;
	if (i2c_smbus_xfer(client->adapter,client->addr,client->flags,
	                   I2C_SMBUS_READ,command, I2C_SMBUS_WORD_DATA, &data))
		return -1;
	else
		return data.word;
}

extern s32 i2c_smbus_write_word_data(struct i2c_client * client,
                                     u8 command, u16 value)
{
	union i2c_smbus_data data;
	data.word = value;
	return i2c_smbus_xfer(client->adapter,client->addr,client->flags,
	                      I2C_SMBUS_WRITE,command,
	                      I2C_SMBUS_WORD_DATA,&data);
}

extern s32 i2c_smbus_process_call(struct i2c_client * client,
                                  u8 command, u16 value)
{
	union i2c_smbus_data data;
	data.word = value;
	if (i2c_smbus_xfer(client->adapter,client->addr,client->flags,
	                   I2C_SMBUS_WRITE,command,
	                   I2C_SMBUS_PROC_CALL, &data))
		return -1;
	else
		return data.word;
}

/* Returns the number of read bytes */
extern s32 i2c_smbus_read_block_data(struct i2c_client * client,
                                     u8 command, u8 *values)
{
	union i2c_smbus_data data;
	int i;
	if (i2c_smbus_xfer(client->adapter,client->addr,client->flags,
	                   I2C_SMBUS_READ,command,
	                   I2C_SMBUS_BLOCK_DATA,&data))
		return -1;
	else {
		for (i = 1; i <= data.block[0]; i++)
			values[i-1] = data.block[i];
		return data.block[0];
	}
}

extern s32 i2c_smbus_write_block_data(struct i2c_client * client,
                                      u8 command, u8 length, const u8 *values)
{
	union i2c_smbus_data data;
	int i;
	if (length > I2C_SMBUS_BLOCK_MAX)
		length = I2C_SMBUS_BLOCK_MAX;
	for (i = 1; i <= length; i++)
		data.block[i] = values[i-1];
	data.block[0] = length;
	return i2c_smbus_xfer(client->adapter,client->addr,client->flags,
	                      I2C_SMBUS_WRITE,command,
	                      I2C_SMBUS_BLOCK_DATA,&data);
}

/* Returns the number of read bytes */
extern s32 i2c_smbus_block_process_call(struct i2c_client * client,
                                        u8 command, u8 length, u8 *values)
{
	union i2c_smbus_data data;
	int i;
	if (length > I2C_SMBUS_BLOCK_MAX - 1)
		return -1;
	data.block[0] = length;
	for (i = 1; i <= length; i++)
		data.block[i] = values[i-1];
	if(i2c_smbus_xfer(client->adapter,client->addr,client->flags,
	                  I2C_SMBUS_WRITE, command,
	                  I2C_SMBUS_BLOCK_PROC_CALL, &data))
		return -1;
	for (i = 1; i <= data.block[0]; i++)
		values[i-1] = data.block[i];
	return data.block[0];
}

/* Returns the number of read bytes */
extern s32 i2c_smbus_read_i2c_block_data(struct i2c_client * client,
                                         u8 command, u8 *values)
{
	union i2c_smbus_data data;
	int i;
	if (i2c_smbus_xfer(client->adapter,client->addr,client->flags,
	                      I2C_SMBUS_READ,command,
	                      I2C_SMBUS_I2C_BLOCK_DATA,&data))
		return -1;
	else {
		for (i = 1; i <= data.block[0]; i++)
			values[i-1] = data.block[i];
		return data.block[0];
	}
}

extern s32 i2c_smbus_write_i2c_block_data(struct i2c_client * client,
                                          u8 command, u8 length,
                                          const u8 *values)
{
	union i2c_smbus_data data;
	int i;
	if (length > I2C_SMBUS_I2C_BLOCK_MAX)
		length = I2C_SMBUS_I2C_BLOCK_MAX;
	for (i = 1; i <= length; i++)
		data.block[i] = values[i-1];
	data.block[0] = length;
	return i2c_smbus_xfer(client->adapter,client->addr,client->flags,
	                      I2C_SMBUS_WRITE,command,
	                      I2C_SMBUS_I2C_BLOCK_DATA,&data);
}

/* Simulate a SMBus command using the i2c protocol 
   No checking of parameters is done!  */
static s32 i2c_smbus_xfer_emulated(struct i2c_adapter * adapter, u16 addr, 
                                   unsigned short flags,
                                   char read_write, u8 command, int size, 
                                   union i2c_smbus_data * data)
{
	/* So we need to generate a series of msgs. In the case of writing, we
	  need to use only one message; when reading, we need two. We initialize
	  most things with sane defaults, to keep the code below somewhat
	  simpler. */
	unsigned char msgbuf0[I2C_SMBUS_BLOCK_MAX+3];
	unsigned char msgbuf1[I2C_SMBUS_BLOCK_MAX+2];
	int num = read_write == I2C_SMBUS_READ?2:1;
	struct i2c_msg msg[2] = { { addr, flags, 1, msgbuf0 }, 
	                          { addr, flags | I2C_M_RD, 0, msgbuf1 }
	                        };
	int i, len;
	u8 partial_pec = 0;

	msgbuf0[0] = command;
	switch(size) {
	case I2C_SMBUS_QUICK:
		msg[0].len = 0;
		/* Special case: The read/write field is used as data */
		msg[0].flags = flags | (read_write == I2C_SMBUS_READ ?
					I2C_M_RD : 0);
		num = 1;
		break;
	case I2C_SMBUS_BYTE:
		if (read_write == I2C_SMBUS_READ) {
			/* Special case: only a read! */
			msg[0].flags = I2C_M_RD | flags;
			num = 1;
		}
		break;
	case I2C_SMBUS_BYTE_DATA:
		if (read_write == I2C_SMBUS_READ)
			msg[1].len = 1;
		else {
			msg[0].len = 2;
			msgbuf0[1] = data->byte;
		}
		break;
	case I2C_SMBUS_WORD_DATA:
		if (read_write == I2C_SMBUS_READ)
			msg[1].len = 2;
		else {
			msg[0].len=3;
			msgbuf0[1] = data->word & 0xff;
			msgbuf0[2] = data->word >> 8;
		}
		break;
	case I2C_SMBUS_PROC_CALL:
		num = 2; /* Special case */
		read_write = I2C_SMBUS_READ;
		msg[0].len = 3;
		msg[1].len = 2;
		msgbuf0[1] = data->word & 0xff;
		msgbuf0[2] = data->word >> 8;
		break;
	case I2C_SMBUS_BLOCK_DATA:
		if (read_write == I2C_SMBUS_READ) {
			/* I2C_FUNC_SMBUS_EMUL doesn't include I2C_FUNC_SMBUS_READ_BLOCK_DATA */
			if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_READ_BLOCK_DATA)) {
				printk(KERN_ERR "i2c-core.o: Block read not supported "
				       "under I2C emulation!\n");
				return -1;
			}
			/* set send message */
			msg[0].len = 1;
			/* set recv message */
			msg[1].flags |= I2C_M_RECV_LEN;
			msg[1].len = I2C_SMBUS_BLOCK_MAX + 1;
			if (flags & I2C_CLIENT_PEC)
				msg[1].flags |= I2C_M_RECV_PEC;
		} else {
			msg[0].len = data->block[0] + 2;
			if (msg[0].len > I2C_SMBUS_BLOCK_MAX + 2) {
				printk(KERN_ERR "i2c-core.o: smbus_access called with "
				       "invalid block write size (%d)\n",
				       data->block[0]);
				return -1;
			}
			for (i = 1; i < msg[0].len; i++)
				msgbuf0[i] = data->block[i-1];
		}
		break;
	case I2C_SMBUS_BLOCK_PROC_CALL:
		/* I2C_FUNC_SMBUS_EMUL doesn't include I2C_FUNC_SMBUS_BLOCK_PROC_CALL */
		if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BLOCK_PROC_CALL)) {
			printk(KERN_ERR "i2c-core.o: adapter doesn't support block process call!\n");
			return -1;
		}

		/* Another special case */
		num = 2;
		read_write = I2C_SMBUS_READ;
		
		/* set send message */
		msg[0].len = data->block[0] + 2;
		if (msg[0].len > I2C_SMBUS_BLOCK_MAX + 2) {
			printk(KERN_ERR "i2c-core.o: smbus_access called with "
				"invalid block write size (%d)\n", data->block[0]);
			return -1;
		}
		for (i = 1; i < msg[0].len; i++)
			msgbuf0[i] = data->block[i-1];
		
		/* set recv message */
		msg[1].flags |= I2C_M_RECV_LEN;
		msg[1].len = I2C_SMBUS_BLOCK_MAX + 1;
		if (flags & I2C_CLIENT_PEC)
			msg[1].flags |= I2C_M_RECV_PEC;
		break;
	case I2C_SMBUS_I2C_BLOCK_DATA:
		if (read_write == I2C_SMBUS_READ) {
			msg[1].len = I2C_SMBUS_I2C_BLOCK_MAX;
		} else {
			msg[0].len = data->block[0] + 1;
			if (msg[0].len > I2C_SMBUS_I2C_BLOCK_MAX + 1) {
				printk("i2c-core.o: i2c_smbus_xfer_emulated called with "
				       "invalid block write size (%d)\n",
				       data->block[0]);
				return -1;
			}
			for (i = 1; i <= data->block[0]; i++)
				msgbuf0[i] = data->block[i];
		}
		break;
	default:
		printk(KERN_ERR "i2c-core.o: smbus_access called with invalid size (%d)\n",
		       size);
		return -1;
	}

	i = ((flags & I2C_CLIENT_PEC) && size != I2C_SMBUS_QUICK
				      && size != I2C_SMBUS_I2C_BLOCK_DATA);
	if (i) {
		/* Compute PEC if first message is a write */
		if (!(msg[0].flags & I2C_M_RD)) {
		 	if (num == 1) /* Write only */
				i2c_smbus_add_pec(&msg[0]);
			else /* Write followed by read */
				partial_pec = i2c_smbus_msg_pec(0, &msg[0]);
		}
		/* Ask for PEC if last message is a read */
		if (msg[num-1].flags & I2C_M_RD)
		 	msg[num-1].len++;
	}

	if (i2c_transfer(adapter, msg, num) < 0)
		return -1;

	/* Check PEC if last message is a read */
	if (i && (msg[num-1].flags & I2C_M_RD)) {
		if (i2c_smbus_check_pec(partial_pec, &msg[num-1]) < 0)
			return -1;
	}

	if (read_write == I2C_SMBUS_READ)
		switch(size) {
			case I2C_SMBUS_BYTE:
				data->byte = msgbuf0[0];
				break;
			case I2C_SMBUS_BYTE_DATA:
				data->byte = msgbuf1[0];
				break;
			case I2C_SMBUS_WORD_DATA: 
			case I2C_SMBUS_PROC_CALL:
				data->word = msgbuf1[0] | (msgbuf1[1] << 8);
				break;
			case I2C_SMBUS_I2C_BLOCK_DATA:
				/* fixed at 32 for now */
				data->block[0] = I2C_SMBUS_I2C_BLOCK_MAX;
				for (i = 0; i < I2C_SMBUS_I2C_BLOCK_MAX; i++)
					data->block[i+1] = msgbuf1[i];
				break;
			case I2C_SMBUS_BLOCK_DATA:
			case I2C_SMBUS_BLOCK_PROC_CALL:
				len = msgbuf1[0] + 1;
				for (i = 0; i < len; i++)
					data->block[i] = msgbuf1[i];
				break;
		}
	return 0;
}


s32 i2c_smbus_xfer(struct i2c_adapter * adapter, u16 addr, unsigned short flags,
                   char read_write, u8 command, int size, 
                   union i2c_smbus_data * data)
{
	s32 res;

	flags &= I2C_M_TEN | I2C_CLIENT_PEC;

	if (adapter->algo->smbus_xfer) {
		I2C_LOCK(adapter);
		res = adapter->algo->smbus_xfer(adapter,addr,flags,read_write,
		                                command,size,data);
		I2C_UNLOCK(adapter);
	} else
		res = i2c_smbus_xfer_emulated(adapter,addr,flags,read_write,
	                                      command,size,data);

	return res;
}


/* You should always define `functionality'; the 'else' is just for
   backward compatibility. */ 
u32 i2c_get_functionality (struct i2c_adapter *adap)
{
	if (adap->algo->functionality)
		return adap->algo->functionality(adap);
	else
		return 0xffffffff;
}

int i2c_check_functionality (struct i2c_adapter *adap, u32 func)
{
	u32 adap_func = i2c_get_functionality (adap);
	return (func & adap_func) == func;
}


static int __init i2c_init(void)
{
	printk(KERN_INFO "i2c-core.o: i2c core module version %s (%s)\n", I2C_VERSION, I2C_DATE);
	memset(adapters,0,sizeof(adapters));
	memset(drivers,0,sizeof(drivers));

#ifdef CONFIG_PROC_FS
	return i2cproc_init();
#else
	return 0;
#endif
}

static void __exit i2c_exit(void) 
{
#ifdef CONFIG_PROC_FS
	i2cproc_cleanup();
#endif
}

/* leave this in for now simply to make patching easier so we don't have
   to remove the call in drivers/char/mem.c */
int __init i2c_init_all(void)
{
	return 0;
}

EXPORT_SYMBOL(i2c_add_adapter);
EXPORT_SYMBOL(i2c_del_adapter);
EXPORT_SYMBOL(i2c_add_driver);
EXPORT_SYMBOL(i2c_del_driver);
EXPORT_SYMBOL(i2c_attach_client);
EXPORT_SYMBOL(i2c_detach_client);
EXPORT_SYMBOL(i2c_get_client);
EXPORT_SYMBOL(i2c_use_client);
EXPORT_SYMBOL(i2c_release_client);
EXPORT_SYMBOL(i2c_check_addr);


EXPORT_SYMBOL(i2c_master_send);
EXPORT_SYMBOL(i2c_master_recv);
EXPORT_SYMBOL(i2c_control);
EXPORT_SYMBOL(i2c_transfer);
EXPORT_SYMBOL(i2c_adapter_id);
EXPORT_SYMBOL(i2c_probe);

EXPORT_SYMBOL(i2c_smbus_xfer);
EXPORT_SYMBOL(i2c_smbus_write_quick);
EXPORT_SYMBOL(i2c_smbus_read_byte);
EXPORT_SYMBOL(i2c_smbus_write_byte);
EXPORT_SYMBOL(i2c_smbus_read_byte_data);
EXPORT_SYMBOL(i2c_smbus_write_byte_data);
EXPORT_SYMBOL(i2c_smbus_read_word_data);
EXPORT_SYMBOL(i2c_smbus_write_word_data);
EXPORT_SYMBOL(i2c_smbus_process_call);
EXPORT_SYMBOL(i2c_smbus_read_block_data);
EXPORT_SYMBOL(i2c_smbus_write_block_data);
EXPORT_SYMBOL(i2c_smbus_read_i2c_block_data);
EXPORT_SYMBOL(i2c_smbus_write_i2c_block_data);

EXPORT_SYMBOL(i2c_get_functionality);
EXPORT_SYMBOL(i2c_check_functionality);

MODULE_AUTHOR("Simon G. Vogl <simon@tk.uni-linz.ac.at>");
MODULE_DESCRIPTION("I2C-Bus main module");
MODULE_LICENSE("GPL");

MODULE_PARM(i2c_debug, "i");
MODULE_PARM_DESC(i2c_debug,"debug level");

module_init(i2c_init);
module_exit(i2c_exit);
