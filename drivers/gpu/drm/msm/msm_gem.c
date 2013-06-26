/*
 * Copyright (C) 2013 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/spinlock.h>
#include <linux/shmem_fs.h>

#include "msm_drv.h"

struct msm_gem_object {
	struct drm_gem_object base;

	struct list_head mm_list;

	uint32_t flags;
	struct page **pages;
	struct sg_table *sgt;
	void *vaddr;

	struct {
		// XXX
		uint32_t iova;
	} domain[NUM_DOMAINS];
};
#define to_msm_bo(x) container_of(x, struct msm_gem_object, base)

/* called with dev->struct_mutex held */
/* TODO move this into drm_gem.c */
static struct page **attach_pages(struct drm_gem_object *obj)
{
	struct inode *inode;
	struct address_space *mapping;
	struct page *p, **pages;
	int i, npages;

	/* This is the shared memory object that backs the GEM resource */
	inode = file_inode(obj->filp);
	mapping = inode->i_mapping;

	npages = obj->size >> PAGE_SHIFT;

	pages = drm_malloc_ab(npages, sizeof(struct page *));
	if (pages == NULL)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < npages; i++) {
		p = shmem_read_mapping_page(mapping, i);
		if (IS_ERR(p))
			goto fail;
		pages[i] = p;
	}

	return pages;

fail:
	while (i--)
		page_cache_release(pages[i]);

	drm_free_large(pages);
	return ERR_CAST(p);
}

static void detach_pages(struct drm_gem_object *obj, struct page **pages)
{
	int i, npages;

	npages = obj->size >> PAGE_SHIFT;

	for (i = 0; i < npages; i++) {
		set_page_dirty(pages[i]);

		/* Undo the reference we took when populating the table */
		page_cache_release(pages[i]);
	}

	drm_free_large(pages);
}


/* called with dev->struct_mutex held */
static struct page **get_pages(struct drm_gem_object *obj)
{
	struct msm_gem_object *msm_obj = to_msm_bo(obj);

	if (!msm_obj->pages) {
		struct page **p = attach_pages(obj);
		int npages = obj->size >> PAGE_SHIFT;

		if (IS_ERR(p)) {
			dev_err(obj->dev->dev, "could not get pages: %ld\n",
					PTR_ERR(p));
			return p;
		}
		msm_obj->pages = p;
		msm_obj->sgt = drm_prime_pages_to_sg(p, npages);
	}

	return msm_obj->pages;
}

static void put_pages(struct drm_gem_object *obj)
{
	struct msm_gem_object *msm_obj = to_msm_bo(obj);

	if (!msm_obj->pages) {
		if (msm_obj->sgt) {
			sg_free_table(msm_obj->sgt);
			kfree(msm_obj->sgt);
		}
		detach_pages(obj, msm_obj->pages);
		msm_obj->pages = NULL;
	}
}

int msm_gem_mmap_obj(struct drm_gem_object *obj,
		struct vm_area_struct *vma)
{
	struct msm_gem_object *msm_obj = to_msm_bo(obj);

	vma->vm_flags &= ~VM_PFNMAP;
	vma->vm_flags |= VM_MIXEDMAP;

	if (msm_obj->flags & MSM_BO_WC) {
		vma->vm_page_prot = pgprot_writecombine(vm_get_page_prot(vma->vm_flags));
	} else if (msm_obj->flags & MSM_BO_UNCACHED) {
		vma->vm_page_prot = pgprot_noncached(vm_get_page_prot(vma->vm_flags));
	} else {
		/*
		 * Shunt off cached objs to shmem file so they have their own
		 * address_space (so unmap_mapping_range does what we want,
		 * in particular in the case of mmap'd dmabufs)
		 */
		fput(vma->vm_file);
		get_file(obj->filp);
		vma->vm_pgoff = 0;
		vma->vm_file  = obj->filp;

		vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);
	}

	return 0;
}

int msm_gem_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret;

	ret = drm_gem_mmap(filp, vma);
	if (ret) {
		DBG("mmap failed: %d", ret);
		return ret;
	}

	return msm_gem_mmap_obj(vma->vm_private_data, vma);
}

int msm_gem_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	struct drm_gem_object *obj = vma->vm_private_data;
	struct msm_gem_object *msm_obj = to_msm_bo(obj);
	struct drm_device *dev = obj->dev;
	struct page **pages;
	unsigned long pfn;
	pgoff_t pgoff;
	int ret;

	/* Make sure we don't parallel update on a fault, nor move or remove
	 * something from beneath our feet
	 */
	mutex_lock(&dev->struct_mutex);

	/* make sure we have pages attached now */
	pages = get_pages(obj);
	if (IS_ERR(pages)) {
		ret = PTR_ERR(pages);
		goto out;
	}

	/* We don't use vmf->pgoff since that has the fake offset: */
	pgoff = ((unsigned long)vmf->virtual_address -
			vma->vm_start) >> PAGE_SHIFT;

	pfn = page_to_pfn(msm_obj->pages[pgoff]);

	VERB("Inserting %p pfn %lx, pa %lx", vmf->virtual_address,
			pfn, pfn << PAGE_SHIFT);

	ret = vm_insert_mixed(vma, (unsigned long)vmf->virtual_address, pfn);

out:
	mutex_unlock(&dev->struct_mutex);
	switch (ret) {
	case 0:
	case -ERESTARTSYS:
	case -EINTR:
		return VM_FAULT_NOPAGE;
	case -ENOMEM:
		return VM_FAULT_OOM;
	default:
		return VM_FAULT_SIGBUS;
	}
}

/** get mmap offset */
static uint64_t mmap_offset(struct drm_gem_object *obj)
{
	struct drm_device *dev = obj->dev;

	WARN_ON(!mutex_is_locked(&dev->struct_mutex));

	if (!obj->map_list.map) {
		/* Make it mmapable */
		int ret = drm_gem_create_mmap_offset(obj);

		if (ret) {
			dev_err(dev->dev, "could not allocate mmap offset\n");
			return 0;
		}
	}

	return (uint64_t)obj->map_list.hash.key << PAGE_SHIFT;
}

uint64_t msm_gem_mmap_offset(struct drm_gem_object *obj)
{
	uint64_t offset;
	mutex_lock(&obj->dev->struct_mutex);
	offset = mmap_offset(obj);
	mutex_unlock(&obj->dev->struct_mutex);
	return offset;
}

int msm_gem_get_iova(struct drm_gem_object *obj, int id, uint32_t *iova)
{
	struct msm_gem_object *msm_obj = to_msm_bo(obj);
	int ret = 0;

	mutex_lock(&obj->dev->struct_mutex);
	if (!msm_obj->domain[id].iova) {
		struct msm_drm_private *priv = obj->dev->dev_private;
		uint32_t offset = (uint32_t)mmap_offset(obj);
		get_pages(obj);
		ret = iommu_map_range(priv->iommus[id], offset,
				msm_obj->sgt->sgl, obj->size, IOMMU_READ);
		msm_obj->domain[id].iova = offset;
	}
	mutex_unlock(&obj->dev->struct_mutex);

	if (!ret)
		*iova = msm_obj->domain[id].iova;

	return ret;
}

void msm_gem_put_iova(struct drm_gem_object *obj, int id)
{
}

int msm_gem_dumb_create(struct drm_file *file, struct drm_device *dev,
		struct drm_mode_create_dumb *args)
{
	args->pitch = align_pitch(args->width, args->bpp);
	args->size  = PAGE_ALIGN(args->pitch * args->height);
	return msm_gem_new_handle(dev, file, args->size,
			MSM_BO_SCANOUT | MSM_BO_WC, &args->handle);
}

int msm_gem_dumb_destroy(struct drm_file *file, struct drm_device *dev,
		uint32_t handle)
{
	/* No special work needed, drop the reference and see what falls out */
	return drm_gem_handle_delete(file, handle);
}

int msm_gem_dumb_map_offset(struct drm_file *file, struct drm_device *dev,
		uint32_t handle, uint64_t *offset)
{
	struct drm_gem_object *obj;
	int ret = 0;

	/* GEM does all our handle to object mapping */
	obj = drm_gem_object_lookup(dev, file, handle);
	if (obj == NULL) {
		ret = -ENOENT;
		goto fail;
	}

	*offset = msm_gem_mmap_offset(obj);

	drm_gem_object_unreference_unlocked(obj);

fail:
	return ret;
}

void *msm_gem_vaddr(struct drm_gem_object *obj)
{
	struct msm_gem_object *msm_obj = to_msm_bo(obj);
	WARN_ON(!mutex_is_locked(&obj->dev->struct_mutex));
	if (!msm_obj->vaddr) {
		struct page **pages = get_pages(obj);
		if (IS_ERR(pages))
			return ERR_CAST(pages);
		msm_obj->vaddr = vmap(pages, obj->size >> PAGE_SHIFT,
				VM_MAP, pgprot_writecombine(PAGE_KERNEL));
	}
	return msm_obj->vaddr;
}

#ifdef CONFIG_DEBUG_FS
void msm_gem_describe(struct drm_gem_object *obj, struct seq_file *m)
{
	struct drm_device *dev = obj->dev;
	struct msm_gem_object *msm_obj = to_msm_bo(obj);
	uint64_t off = 0;

	WARN_ON(!mutex_is_locked(&dev->struct_mutex));

	if (obj->map_list.map)
		off = (uint64_t)obj->map_list.hash.key;

	seq_printf(m, "%08x: %2d (%2d) %08llx %p %d\n",
			msm_obj->flags, obj->name, obj->refcount.refcount.counter,
			off, msm_obj->vaddr, obj->size);
}

void msm_gem_describe_objects(struct list_head *list, struct seq_file *m)
{
	struct msm_gem_object *msm_obj;
	int count = 0;
	size_t size = 0;

	list_for_each_entry(msm_obj, list, mm_list) {
		struct drm_gem_object *obj = &msm_obj->base;
		seq_printf(m, "   ");
		msm_gem_describe(obj, m);
		count++;
		size += obj->size;
	}

	seq_printf(m, "Total %d objects, %zu bytes\n", count, size);
}
#endif

void msm_gem_free_object(struct drm_gem_object *obj)
{
	struct drm_device *dev = obj->dev;
	struct msm_gem_object *msm_obj = to_msm_bo(obj);
	int id;

	WARN_ON(!mutex_is_locked(&dev->struct_mutex));

	list_del(&msm_obj->mm_list);

	if (obj->map_list.map)
		drm_gem_free_mmap_offset(obj);

	if (msm_obj->vaddr)
		vunmap(msm_obj->vaddr);

	for (id = 0; id < ARRAY_SIZE(msm_obj->domain); id++) {
		if (msm_obj->domain[id].iova) {
			struct msm_drm_private *priv = obj->dev->dev_private;
			uint32_t offset = (uint32_t)mmap_offset(obj);
			iommu_unmap_range(priv->iommus[id], offset, obj->size);
		}
	}

	put_pages(obj);

	drm_gem_object_release(obj);

	kfree(obj);
}

/* convenience method to construct a GEM buffer object, and userspace handle */
int msm_gem_new_handle(struct drm_device *dev, struct drm_file *file,
		uint32_t size, uint32_t flags, uint32_t *handle)
{
	struct drm_gem_object *obj;
	int ret;

	obj = msm_gem_new(dev, size, flags);
	if (!obj)
		return -ENOMEM;

	ret = drm_gem_handle_create(file, obj, handle);

	/* drop reference from allocate - handle holds it now */
	drm_gem_object_unreference_unlocked(obj);

	return ret;
}

struct drm_gem_object *msm_gem_new(struct drm_device *dev,
		uint32_t size, uint32_t flags)
{
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_gem_object *msm_obj;
	struct drm_gem_object *obj = NULL;
	int ret;

	size = PAGE_ALIGN(size);

	msm_obj = kzalloc(sizeof(*msm_obj), GFP_KERNEL);
	if (!msm_obj)
		goto fail;

	obj = &msm_obj->base;

	ret = drm_gem_object_init(dev, obj, size);
	if (ret)
		goto fail;

	msm_obj->flags = flags;

	mutex_lock(&obj->dev->struct_mutex);
	list_add(&msm_obj->mm_list, &priv->obj_list);
	mutex_unlock(&obj->dev->struct_mutex);

	return obj;

fail:
	if (obj)
		drm_gem_object_unreference_unlocked(obj);

	return NULL;
}
