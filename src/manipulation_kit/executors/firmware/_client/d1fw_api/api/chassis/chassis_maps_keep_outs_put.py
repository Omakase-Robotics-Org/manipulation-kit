from http import HTTPStatus
from typing import Any
from urllib.parse import quote

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.chassis_maps_keep_outs_put_response_200 import (
    ChassisMapsKeepOutsPutResponse200,
)
from ...models.error_envelope import ErrorEnvelope
from ...models.keep_outs_put import KeepOutsPut
from ...types import Response


def _get_kwargs(
    scene: str,
    *,
    body: KeepOutsPut,
) -> dict[str, Any]:
    headers: dict[str, Any] = {}

    _kwargs: dict[str, Any] = {
        "method": "put",
        "url": "/v1/chassis/maps/{scene}/keep_outs".format(
            scene=quote(str(scene), safe=""),
        ),
    }

    _kwargs["json"] = body.to_dict()

    headers["Content-Type"] = "application/json"

    _kwargs["headers"] = headers
    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ChassisMapsKeepOutsPutResponse200 | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ChassisMapsKeepOutsPutResponse200.from_dict(response.json())

        return response_200

    if response.status_code == 400:
        response_400 = ErrorEnvelope.from_dict(response.json())

        return response_400

    if response.status_code == 404:
        response_404 = ErrorEnvelope.from_dict(response.json())

        return response_404

    if response.status_code == 409:
        response_409 = ErrorEnvelope.from_dict(response.json())

        return response_409

    if response.status_code == 502:
        response_502 = ErrorEnvelope.from_dict(response.json())

        return response_502

    if response.status_code == 504:
        response_504 = ErrorEnvelope.from_dict(response.json())

        return response_504

    if client.raise_on_unexpected_status:
        raise errors.UnexpectedStatus(response.status_code, response.content)
    else:
        return None


def _build_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> Response[ChassisMapsKeepOutsPutResponse200 | ErrorEnvelope]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    scene: str,
    *,
    client: AuthenticatedClient | Client,
    body: KeepOutsPut,
) -> Response[ChassisMapsKeepOutsPutResponse200 | ErrorEnvelope]:
    """Replace one map's keep-out areas

     Whole-sub-resource replacement, and therefore idempotent: `forbidden_areas` and `splice_areas` are
    the scene's complete keep-out and typed-zone geometry afterwards, not a delta, so writing both empty
    clears the scene. Gated by the chassis soft-kill latch, because a keep-out area is a place the base
    must not drive. This is the ONE map write the base serves behind its own login aspect; the daemon's
    built-in `sdkKey` (overridable through `[chassis] sdk_key`) satisfies it, and a base that still
    refuses reports its refusal in the envelope's `message`. `data` is the base's own reply, forwarded
    unchanged.

    Args:
        scene (str):
        body (KeepOutsPut): `PUT /v1/chassis/maps/{scene}/keep_outs`.

            Whole-sub-resource replacement, matching [`RoadPut`]'s semantics: the two
            fields are the scene's complete keep-out and typed-zone geometry
            afterwards, not a delta, so writing both empty clears the scene. Each
            point in `forbidden_areas` and in a [`SpliceAreaPut`]'s `points` accepts
            either the base's own `[x, y]` array spelling or the `{"x": x, "y": y}`
            object spelling the keep-out `GET` serves inside each
            `forbiddenAreasTemp` string (`d1fw_core::Point`), so a caller that echoes
            a point it just read back into a write is accepted too.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisMapsKeepOutsPutResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        scene=scene,
        body=body,
    )

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    scene: str,
    *,
    client: AuthenticatedClient | Client,
    body: KeepOutsPut,
) -> ChassisMapsKeepOutsPutResponse200 | ErrorEnvelope | None:
    """Replace one map's keep-out areas

     Whole-sub-resource replacement, and therefore idempotent: `forbidden_areas` and `splice_areas` are
    the scene's complete keep-out and typed-zone geometry afterwards, not a delta, so writing both empty
    clears the scene. Gated by the chassis soft-kill latch, because a keep-out area is a place the base
    must not drive. This is the ONE map write the base serves behind its own login aspect; the daemon's
    built-in `sdkKey` (overridable through `[chassis] sdk_key`) satisfies it, and a base that still
    refuses reports its refusal in the envelope's `message`. `data` is the base's own reply, forwarded
    unchanged.

    Args:
        scene (str):
        body (KeepOutsPut): `PUT /v1/chassis/maps/{scene}/keep_outs`.

            Whole-sub-resource replacement, matching [`RoadPut`]'s semantics: the two
            fields are the scene's complete keep-out and typed-zone geometry
            afterwards, not a delta, so writing both empty clears the scene. Each
            point in `forbidden_areas` and in a [`SpliceAreaPut`]'s `points` accepts
            either the base's own `[x, y]` array spelling or the `{"x": x, "y": y}`
            object spelling the keep-out `GET` serves inside each
            `forbiddenAreasTemp` string (`d1fw_core::Point`), so a caller that echoes
            a point it just read back into a write is accepted too.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisMapsKeepOutsPutResponse200 | ErrorEnvelope
    """

    return sync_detailed(
        scene=scene,
        client=client,
        body=body,
    ).parsed


async def asyncio_detailed(
    scene: str,
    *,
    client: AuthenticatedClient | Client,
    body: KeepOutsPut,
) -> Response[ChassisMapsKeepOutsPutResponse200 | ErrorEnvelope]:
    """Replace one map's keep-out areas

     Whole-sub-resource replacement, and therefore idempotent: `forbidden_areas` and `splice_areas` are
    the scene's complete keep-out and typed-zone geometry afterwards, not a delta, so writing both empty
    clears the scene. Gated by the chassis soft-kill latch, because a keep-out area is a place the base
    must not drive. This is the ONE map write the base serves behind its own login aspect; the daemon's
    built-in `sdkKey` (overridable through `[chassis] sdk_key`) satisfies it, and a base that still
    refuses reports its refusal in the envelope's `message`. `data` is the base's own reply, forwarded
    unchanged.

    Args:
        scene (str):
        body (KeepOutsPut): `PUT /v1/chassis/maps/{scene}/keep_outs`.

            Whole-sub-resource replacement, matching [`RoadPut`]'s semantics: the two
            fields are the scene's complete keep-out and typed-zone geometry
            afterwards, not a delta, so writing both empty clears the scene. Each
            point in `forbidden_areas` and in a [`SpliceAreaPut`]'s `points` accepts
            either the base's own `[x, y]` array spelling or the `{"x": x, "y": y}`
            object spelling the keep-out `GET` serves inside each
            `forbiddenAreasTemp` string (`d1fw_core::Point`), so a caller that echoes
            a point it just read back into a write is accepted too.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisMapsKeepOutsPutResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        scene=scene,
        body=body,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    scene: str,
    *,
    client: AuthenticatedClient | Client,
    body: KeepOutsPut,
) -> ChassisMapsKeepOutsPutResponse200 | ErrorEnvelope | None:
    """Replace one map's keep-out areas

     Whole-sub-resource replacement, and therefore idempotent: `forbidden_areas` and `splice_areas` are
    the scene's complete keep-out and typed-zone geometry afterwards, not a delta, so writing both empty
    clears the scene. Gated by the chassis soft-kill latch, because a keep-out area is a place the base
    must not drive. This is the ONE map write the base serves behind its own login aspect; the daemon's
    built-in `sdkKey` (overridable through `[chassis] sdk_key`) satisfies it, and a base that still
    refuses reports its refusal in the envelope's `message`. `data` is the base's own reply, forwarded
    unchanged.

    Args:
        scene (str):
        body (KeepOutsPut): `PUT /v1/chassis/maps/{scene}/keep_outs`.

            Whole-sub-resource replacement, matching [`RoadPut`]'s semantics: the two
            fields are the scene's complete keep-out and typed-zone geometry
            afterwards, not a delta, so writing both empty clears the scene. Each
            point in `forbidden_areas` and in a [`SpliceAreaPut`]'s `points` accepts
            either the base's own `[x, y]` array spelling or the `{"x": x, "y": y}`
            object spelling the keep-out `GET` serves inside each
            `forbiddenAreasTemp` string (`d1fw_core::Point`), so a caller that echoes
            a point it just read back into a write is accepted too.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisMapsKeepOutsPutResponse200 | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            scene=scene,
            client=client,
            body=body,
        )
    ).parsed
